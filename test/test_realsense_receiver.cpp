// Runs one RealsenseReceiver per configured camera — subscribes, decodes, and
// reports received color/depth fps per camera once per second (headless). This
// is how a consumer app embeds RealsenseReceiver, minus the fps print: start
// one per camera, then read color()/depth() in-process.
//   ./test_realsense_receiver [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/realsense_receiver.hpp"
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
struct CamRx {
    std::string name;
    std::unique_ptr<kist::RealsenseReceiver> rx;
    uint64_t last_c = 0, last_d = 0;   // decode counters at last window
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

    const auto names = kist::camera_names_from_yaml(root);
    if (names.empty()) {
        std::cerr << "[test_realsense_receiver] no cameras in " << config_path << "\n";
        return 1;
    }

    std::vector<CamRx> cams;
    for (const auto& name : names) {
        auto rx = std::make_unique<kist::RealsenseReceiver>();
        if (!rx->start(domain_id, iface, name)) {
            std::cerr << "[test_realsense_receiver] camera '" << name << "' failed — skipped\n";
            continue;
        }
        cams.push_back(CamRx{name, std::move(rx)});
    }
    if (cams.empty()) {
        std::cerr << "[test_realsense_receiver] no cameras started\n";
        return 1;
    }

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_receiver] %zu camera(s) subscribing on domain=%d iface=%s\n",
                cams.size(), domain_id, iface.c_str());

    // Per-second decode-fps per camera from the receiver's produce-site
    // counters (delta of a monotonic count incremented where each frame is
    // decoded), not by polling the latest-wins buffer — so it can't miss a
    // frame that arrives between polls. This is the Rx number for the
    // Tx→Rx→consumer fps comparison.
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (auto& cam : cams) {
            const uint64_t c = cam.rx->color_decoded();
            const uint64_t d = cam.rx->depth_decoded();
            std::printf("  %-12s color %2llu fps  depth %2llu fps\n", cam.name.c_str(),
                        (unsigned long long)(c - cam.last_c),
                        (unsigned long long)(d - cam.last_d));
            cam.last_c = c;
            cam.last_d = d;
        }
    }

    for (auto& cam : cams) cam.rx->stop();
    return 0;
}
