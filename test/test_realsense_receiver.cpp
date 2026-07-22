// Runs the RealsenseReceiver as a standalone process — subscribes, decodes,
// and reports received color/depth fps once per second (headless). This is
// exactly how a consumer app would embed RealsenseReceiver, minus the fps
// print: start it, then read color()/depth() in-process.
//   ./test_realsense_receiver [config_path]      (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/realsense_receiver.hpp"
#include "common/config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
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
    const std::string iface = unitree["network_interface"].as<std::string>("lo");

    kist::RealsenseReceiver rx;
    if (!rx.start(domain_id, iface))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_realsense_receiver] subscribing on domain=%d iface=%s\n",
                domain_id, iface.c_str());

    int color_n = 0, depth_n = 0;
    int64_t last_c = -1, last_d = -1;
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        // Exactly what an embedding consumer does: poll the decoded buffers.
        auto c = rx.color().GetData();
        auto d = rx.depth().GetData();
        if (c && c->stamp_ns != last_c) { last_c = c->stamp_ns; ++color_n; }
        if (d && d->stamp_ns != last_d) { last_d = d->stamp_ns; ++depth_n; }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            std::printf("received  color %2d fps  depth %2d fps%s\n",
                        color_n, depth_n, (c && d) ? "" : "  (waiting)");
            color_n = depth_n = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    rx.stop();
    return 0;
}
