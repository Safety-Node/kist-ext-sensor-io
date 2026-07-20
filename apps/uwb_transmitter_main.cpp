// UWB publisher daemon — runs on the G1 onboard PC (where the DWM
// dongle is plugged in):
//   ./uwb_transmitter [config_path]
//
// serial /dev/uwb -> DWM lec parsing -> DDS rt/kist/uwb/pose.
// Publishes on NEW samples only (unlike the old ROS2 uwb_node, which
// re-published the last sample forever): when the tag loses its fix or
// the serial link dies, the topic goes silent and downstream buffers
// stale — the empty-buffer principle needs honest silence.
// Prints a once-per-second status line. Ctrl-C to stop.

#include "common/config.hpp"
#include "uwb/dwm_serial.hpp"
#include "uwb/uwb_transmitter.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    // Line-buffer stdout even when piped (nohup/systemd/logging) — the
    // once-per-second status must not sit in a full buffer.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const auto unitree_cfg = root["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    const auto interface   = unitree_cfg["network_interface"].as<std::string>();

    const auto uwb_cfg     = root["uwb"];
    const auto serial_port = uwb_cfg["serial_port"].as<std::string>();
    const auto serial_baud = uwb_cfg["serial_baud"].as<int>();
    const auto frame_id    = uwb_cfg["frame_id"].as<std::string>();

    UwbTransmitter publisher;
    if (!publisher.start(domain_id, interface))
        return 1;

    DwmSerial serial;
    serial.start(serial_port, serial_baud);

    // Ctrl-C and service stop (systemd sends SIGTERM) both exit the
    // main loop so the serial thread joins and the port closes cleanly.
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    using clock = std::chrono::steady_clock;
    auto last_sample_time = std::chrono::steady_clock::time_point{};
    auto window_start     = clock::now();
    int  published_in_window = 0;
    UwbSample last;

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto sample = serial.sample_buf.GetDataWithTime();
        if (sample.HasData() && sample.timestamp != last_sample_time) {
            last_sample_time = sample.timestamp;
            last = *sample.data;
            publisher.publish(last, frame_id);
            ++published_in_window;
        }

        if (clock::now() - window_start >= std::chrono::seconds(1)) {
            window_start = clock::now();
            if (published_in_window > 0) {
                std::printf("rate=%2dHz  pos=(%.2f, %.2f, %.2f)  quality=%d\n",
                            published_in_window, last.x, last.y, last.z, last.quality);
            } else {
                std::printf("no fix (nothing published)\n");
            }
            published_in_window = 0;
        }
    }

    serial.stop();
    return 0;
}
