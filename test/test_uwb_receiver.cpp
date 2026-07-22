// Runs the UwbReceiver as a standalone process — subscribes to UWB fixes and
// prints the latest position + receive rate once per second. Mirrors how a
// consumer embeds UwbReceiver: start it, then read fix() in-process.
//   ./test_uwb_receiver [config_path]         (default config/config.yaml)
// Ctrl-C / SIGTERM to stop.

#include "system/uwb_receiver.hpp"
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

    kist::UwbReceiver rx;
    if (!rx.start(domain_id, iface))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_uwb_receiver] subscribing on domain=%d iface=%s\n",
                domain_id, iface.c_str());

    int n = 0;
    int64_t last = -1;
    auto window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto fix = rx.fix().GetData();
        if (fix && fix->stamp_ns != last) { last = fix->stamp_ns; ++n; }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            if (fix)
                std::printf("received %2d fix/s  latest xyz = (%.2f, %.2f, %.2f)\n",
                            n, fix->x, fix->y, fix->z);
            else
                std::printf("received  0 fix/s  (no fix)\n");
            n = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    rx.stop();
    return 0;
}
