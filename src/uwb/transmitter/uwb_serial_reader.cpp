#include "uwb/transmitter/uwb_serial_reader.hpp"

#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace kist {

// ── pure parsing helpers ──────────────────────────────────────────────

std::optional<UwbSample> parse_pos_line(std::string_view line) {
    const auto idx = line.find("POS,");
    if (idx == std::string_view::npos)
        return std::nullopt;

    // split "POS,<x>,<y>,<z>,<quality>[,...]" on commas
    std::string_view rest = line.substr(idx);
    std::array<std::string_view, 5> parts;
    std::size_t n_parts = 0;
    while (n_parts < parts.size()) {
        const auto comma = rest.find(',');
        parts[n_parts++] = rest.substr(0, comma);
        if (comma == std::string_view::npos)
            break;
        rest = rest.substr(comma + 1);
    }
    if (n_parts < 5)
        return std::nullopt;

    const auto to_float = [](std::string_view sv, float& out) {
        char* end = nullptr;
        const std::string s(sv);  // strtof needs NUL termination
        out = std::strtof(s.c_str(), &end);
        return end != s.c_str() && *end == '\0';
    };

    UwbSample sample;
    float quality = 0.0f;
    if (!to_float(parts[1], sample.x) ||
        !to_float(parts[2], sample.y) ||
        !to_float(parts[3], sample.z) ||
        !to_float(parts[4], quality))
        return std::nullopt;

    sample.quality = int(quality);
    if (sample.quality <= 0)
        return std::nullopt;  // quality=0 means no fix
    return sample;
}

std::vector<std::string> extract_lines(std::string& buf) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const auto idx = buf.find("\r\n", start);
        if (idx == std::string::npos)
            break;
        if (idx > start)
            out.emplace_back(buf.substr(start, idx - start));
        start = idx + 2;
    }
    if (start > 0)
        buf.erase(0, start);
    return out;
}

// ── serial transport ──────────────────────────────────────────────────

namespace {

speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:
            throw std::invalid_argument("[UwbSerialReader] unsupported baud rate");
    }
}

void write_all(int fd, std::string_view data) {
    while (!data.empty()) {
        const ssize_t n = ::write(fd, data.data(), data.size());
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR)
                continue;
            throw std::runtime_error("[UwbSerialReader] serial write failed");
        }
        data = data.substr(std::size_t(n));
    }
    ::tcdrain(fd);
}

// Non-blocking read of whatever is pending; returns bytes appended.
std::size_t read_pending(int fd, std::string& buf) {
    char chunk[256];
    std::size_t total = 0;
    while (true) {
        const ssize_t n = ::read(fd, chunk, sizeof(chunk));
        if (n > 0) {
            buf.append(chunk, std::size_t(n));
            total += std::size_t(n);
            continue;
        }
        if (n == 0 || errno == EAGAIN)
            return total;
        if (errno == EINTR)
            continue;
        throw std::runtime_error("[UwbSerialReader] serial read failed (disconnect?)");
    }
}

bool contains_data(const std::string& buf) {
    return buf.find("POS") != std::string::npos ||
           buf.find("DIST") != std::string::npos;
}

} // namespace

void UwbSerialReader::start(const std::string& port, int baud) {
    if (thread_.joinable())
        return;
    port_ = port;
    baud_ = baud;
    to_speed(baud);  // validate early, throws on bad config
    running_ = true;
    thread_ = std::thread(&UwbSerialReader::reader_loop, this);
}

void UwbSerialReader::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// Outer loop — open the port once, re-open only on I/O error
// (physical disconnect), never on parse trouble.
void UwbSerialReader::reader_loop() {
    using namespace std::chrono_literals;
    pthread_setname_np(pthread_self(), "uwb-serial-rd");  // visible in htop/gdb

    while (running_) {
        try {
            if (fd_ < 0) {
                fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd_ < 0)
                    throw std::runtime_error("[UwbSerialReader] cannot open " + port_);

                termios tio{};
                if (::tcgetattr(fd_, &tio) != 0)
                    throw std::runtime_error("[UwbSerialReader] tcgetattr failed");
                ::cfmakeraw(&tio);
                ::cfsetispeed(&tio, to_speed(baud_));
                ::cfsetospeed(&tio, to_speed(baud_));
                tio.c_cflag |= CLOCAL | CREAD;
                tio.c_cflag &= ~HUPCL;  // keep DTR on close — dropping it resets the DWM MCU
                tio.c_cc[VMIN]  = 0;
                tio.c_cc[VTIME] = 0;
                if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
                    throw std::runtime_error("[UwbSerialReader] tcsetattr failed");

                std::this_thread::sleep_for(300ms);
                handshake();
                std::cout << "[UwbSerialReader] streaming on " << port_ << "\n";
            }
            read_loop();
            return;  // stop() requested
        } catch (const std::exception& e) {
            std::cerr << e.what() << " — retrying in 2s\n";
            if (fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;
            }
            for (int i = 0; i < 200 && running_; ++i)
                std::this_thread::sleep_for(10ms);
        }
    }
}

// State-aware shell entry, same branches as the onboard uwb_node:
// already streaming -> nothing to do; at the prompt -> "lec"; otherwise
// \r\r for the prompt (a stream appearing meanwhile is toggled off by
// "lec" so the prompt shows), then "lec".
void UwbSerialReader::handshake() {
    using namespace std::chrono_literals;
    using clock = std::chrono::steady_clock;

    std::string buf;
    read_pending(fd_, buf);

    if (contains_data(buf))
        return;  // lec already streaming

    if (buf.find("dwm>") == std::string::npos) {
        write_all(fd_, "\r");
        std::this_thread::sleep_for(100ms);
        write_all(fd_, "\r");

        buf.clear();
        const auto deadline = clock::now() + 3s;
        while (clock::now() < deadline) {
            if (read_pending(fd_, buf) == 0) {
                std::this_thread::sleep_for(10ms);
                continue;
            }
            if (buf.find("dwm>") != std::string::npos)
                break;
            if (contains_data(buf)) {
                buf.clear();
                write_all(fd_, "lec\r");  // toggles the stream off -> prompt
                std::this_thread::sleep_for(100ms);
            }
        }
        if (buf.find("dwm>") == std::string::npos)
            throw std::runtime_error("[UwbSerialReader] failed to enter DWM shell");
    }

    write_all(fd_, "lec\r");
    std::string ack;
    const auto deadline = clock::now() + 3s;
    while (clock::now() < deadline) {
        if (read_pending(fd_, ack) == 0) {
            std::this_thread::sleep_for(10ms);
            continue;
        }
        if (contains_data(ack) || ack.find("dwm>") != std::string::npos)
            return;
    }
    throw std::runtime_error("[UwbSerialReader] lec command not acknowledged");
}

void UwbSerialReader::read_loop() {
    using namespace std::chrono_literals;

    std::string buf;
    while (running_) {
        if (read_pending(fd_, buf) == 0) {
            std::this_thread::sleep_for(5ms);
            continue;
        }
        for (const auto& line : extract_lines(buf)) {
            if (auto sample = parse_pos_line(line))
                sample_buf.SetData(std::move(*sample));
        }
    }
}

} // namespace kist
