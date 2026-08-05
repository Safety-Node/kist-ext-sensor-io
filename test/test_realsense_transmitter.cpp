// Runs one RealsenseTransmitter per configured camera — reads the `cameras`
// list (via cameras_from_yaml), brings up capture + encode-publishers for each,
// and streams typed DDS frames (namespaced per camera name) until stopped. Pair
// with test_realsense_receiver (or any embedded RealsenseReceiver) to check the
// transport end to end.
//   ./test_realsense_transmitter [config_path]   (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/realsense_transmitter.hpp"
#include "realsense/realsense_config.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_stop{false};

namespace {
struct CamTx {
    std::string name;
    std::unique_ptr<kist::RealsenseTransmitter> tx;
    uint64_t last_c = 0, last_d = 0;   // published counters at last window
};
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    const auto specs = kist::cameras_from_yaml(root);
    if (specs.empty()) {
        std::cerr << "[test_realsense_transmitter] no cameras in " << config_path << "\n";
        return 1;
    }

    std::vector<CamTx> txs;
    for (const auto& s : specs) {
        auto tx = std::make_unique<kist::RealsenseTransmitter>();
        if (!tx->start(domain_id, iface, s.name, s.capture, s.encoder)) {
            std::cerr << "[test_realsense_transmitter] camera '" << s.name
                      << "' failed to start — skipped\n";
            continue;   // one bad camera shouldn't take down the others
        }
        std::printf("[test_realsense_transmitter] '%s' (serial=%s) %dx%d@%d\n",
                    s.name.c_str(), s.capture.serial.empty() ? "auto" : s.capture.serial.c_str(),
                    s.capture.color_width, s.capture.color_height, s.capture.color_fps);
        txs.push_back(CamTx{s.name, std::move(tx)});
    }
    if (txs.empty()) {
        std::cerr << "[test_realsense_transmitter] no cameras started\n";
        return 1;
    }

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_transmitter] %zu camera(s) publishing on domain=%d iface=%s\n",
                txs.size(), domain_id, iface.c_str());

    // Per-second published-fps per camera (produce-site delta of the assembly's
    // monotonic counters). This is the Tx-side number for the Tx→Rx→consumer
    // fps comparison; pair it with test_realsense_receiver's decode fps.
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (auto& cam : txs) {
            const uint64_t c = cam.tx->color_published();
            const uint64_t d = cam.tx->depth_published();
            std::printf("  %-12s color %2llu fps  depth %2llu fps\n", cam.name.c_str(),
                        (unsigned long long)(c - cam.last_c),
                        (unsigned long long)(d - cam.last_d));
            cam.last_c = c;
            cam.last_d = d;
        }
    }

    for (auto& cam : txs) cam.tx->stop();
    return 0;
}
