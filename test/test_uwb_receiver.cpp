// Live UWB stream check (a running Tx daemon needed, same DDS domain —
// on the same machine or across the network):
//   ./test_uwb_receiver [config_path]
// Interface/domain come from config/config.yaml. Prints position age,
// receive rate, and coordinates once per second. Ctrl-C to stop.
//
// Goes through the ExtSensorIoRx facade, so this doubles as the
// consumer-side assembly probe.

#include "system/ext_sensor_io_rx.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // visible when piped/logged

    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";

    std::signal(SIGINT, [](int) { g_stop = true; });

    auto& rx = ExtSensorIoRx::instance();
    if (!rx.start(config_path))
        return 1;
    auto& uwb = rx.uwb();

    int64_t last_stamp = 0;
    int     received = 0;
    auto    window_start = std::chrono::steady_clock::now();

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        auto pos = uwb.uwb_buf.GetDataWithTime();
        if (pos.HasData() && pos.data->stamp_ns != last_stamp) {
            last_stamp = pos.data->stamp_ns;
            ++received;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window_start < std::chrono::seconds(1))
            continue;
        window_start = now;

        if (!pos.HasData()) {
            std::printf("no position (buffer empty)\n");
            received = 0;
            continue;
        }
        std::printf("rate=%2dHz  age=%6.1fms  pos=(%.2f, %.2f, %.2f)\n",
                    received, pos.GetAgeMs(),
                    pos.data->x, pos.data->y, pos.data->z);
        received = 0;
    }

    rx.stop();
    return 0;
}
