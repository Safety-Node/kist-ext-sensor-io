// External-sensor Tx daemon — runs on the device machine (e.g. the G1
// onboard PC) and publishes every sensor enabled in the config:
//   ./ext_sensor_io [config_path]
// Ctrl-C / SIGTERM to stop.

#include "system/ext_sensor_io_tx.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    // Line-buffer stdout even when piped (nohup/systemd/logging) — the
    // once-per-second status must not sit in a full buffer.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    auto& sys = kist::ExtSensorIoTx::instance();
    if (!sys.start(config_path))
        return 1;

    // Ctrl-C and service stop (systemd sends SIGTERM) both exit the
    // main loop so sensor threads join and devices close cleanly.
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    sys.stop();
    return 0;
}
