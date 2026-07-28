// Runs one RealsenseReceiver per configured camera — subscribes, decodes, and
// reports received color/depth fps per camera once per second (headless). This
// is how a consumer app embeds RealsenseReceiver, minus the fps print: start
// one per camera, then read color()/depth() in-process.
//   ./test_realsense_receiver [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/realsense_receiver.hpp"
#include "realsense/realsense_config.hpp"
#include "common/config.hpp"

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
    int64_t last_c = -1, last_d = -1;
    int     color_n = 0, depth_n = 0;
};
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();

    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    const std::string iface = unitree["network_interface"].as<std::string>("lo");

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

    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        for (auto& cam : cams) {
            auto c = cam.rx->color().GetData();
            auto d = cam.rx->depth().GetData();
            if (c && c->stamp_ns != cam.last_c) { cam.last_c = c->stamp_ns; ++cam.color_n; }
            if (d && d->stamp_ns != cam.last_d) { cam.last_d = d->stamp_ns; ++cam.depth_n; }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            for (auto& cam : cams) {
                std::printf("  %-12s color %2d fps  depth %2d fps\n",
                            cam.name.c_str(), cam.color_n, cam.depth_n);
                cam.color_n = cam.depth_n = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    for (auto& cam : cams) cam.rx->stop();
    return 0;
}
