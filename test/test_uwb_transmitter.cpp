// Runs the UwbTransmitter as a standalone process — reads config, opens the
// DWM serial port, parses fixes, and publishes them over DDS until stopped.
//   ./test_uwb_transmitter [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/uwb_transmitter.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    const auto uwb = root["uwb"];
    if (!uwb || !uwb["enabled"].as<bool>(true)) {
        std::cerr << "[test_uwb_transmitter] no enabled 'uwb' section in "
                  << config_path << "\n";
        return 1;
    }
    const auto port     = uwb["serial_port"].as<std::string>("/dev/uwb");
    const int  baud     = uwb["serial_baud"].as<int>(115200);
    const auto frame_id = uwb["frame_id"].as<std::string>("map");

    kist::UwbTransmitter tx;
    if (!tx.start(domain_id, iface, port, baud, frame_id))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_uwb_transmitter] publishing on domain=%d iface=%s port=%s\n",
                domain_id, iface.c_str(), port.c_str());

    // Device-side status: poll the sample buffer and report the publish rate
    // once per second. The library itself stays silent.
    auto last_sample_time = std::chrono::steady_clock::time_point{};
    int published = 0;
    kist::UwbSample last{};
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto s = tx.samples().GetDataWithTime();
        if (s.HasData() && s.timestamp != last_sample_time) {
            last_sample_time = s.timestamp;
            last = *s.data;
            ++published;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            if (published > 0)
                std::printf("[uwb] rate=%2dHz  pos=(%.2f, %.2f, %.2f)  quality=%d\n",
                            published, last.x, last.y, last.z, last.quality);
            else
                std::printf("[uwb] no fix (nothing published)\n");
            published = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tx.stop();
    return 0;
}
