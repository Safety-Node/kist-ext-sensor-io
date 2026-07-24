#pragma once

#include "common/data_buffer.hpp"
#include "uwb/transmitter/uwb_sample.hpp"

#include <atomic>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace kist {

// ── pure parsing helpers (exposed for deterministic tests) ────────────
//
// DWM "lec" CSV lines look like
//   POS,1.10,2.20,0.90,85
// or, with anchor ranges prepended,
//   DIST,4,AN0,...,POS,1.10,2.20,0.90,85
// parse_pos_line finds "POS," anywhere in the line; quality <= 0 means
// "no fix" and yields nullopt.
std::optional<UwbSample> parse_pos_line(std::string_view line);

// Splits complete "\r\n"-terminated lines off the front of buf (partial
// tail is kept for the next chunk).
std::vector<std::string> extract_lines(std::string& buf);

// ── serial transport ──────────────────────────────────────────────────
//
// Decawave DWM UART shell (DWM1001 / DWM3001CDK over USB-serial):
// enter the shell with \r\r, start location-engine CSV streaming with
// "lec", then parse POS lines forever.
//
// The port is opened ONCE and kept open for the lifetime of the object —
// closing and re-opening toggles DTR, which resets the DWM MCU (J-Link
// OB behaviour); HUPCL is also cleared for the same reason. Only a true
// I/O error (physical disconnect) triggers a re-open, after a 2s wait.
//
// Same recovery structure as the onboard uwb_node's SerialTransport.
class UwbSerialReader {
public:
    ~UwbSerialReader() { stop(); }

    void start(const std::string& port, int baud = 115200);
    void stop();

    // Latest valid fix; watch the timestamp to detect new samples.
    DataBuffer<UwbSample> sample_buf;

private:
    void reader_loop();
    void handshake();   // \r\r -> dwm> -> lec (state-aware), throws on timeout
    void read_loop();   // parse POS lines until stop or I/O error (throws)

    std::string       port_;
    int               baud_ = 115200;
    int               fd_ = -1;
    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
