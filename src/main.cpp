// kist_ext_sensor_io — the operational runner: every sensor this machine
// handles, in one process (the per-sensor test_* runners remain for
// isolating a single sensor).
//
//   ./build/kist_ext_sensor_io tx [config]   device side: UWB + all cameras -> DDS
//   ./build/kist_ext_sensor_io rx [config]   consumer side: receive + report rates
//
// Reads config/config.yaml by default; prints per-second per-sensor rates;
// Ctrl-C / SIGTERM stops.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "mic/mic_config.hpp"
#include "realsense/realsense_config.hpp"
#include "system/ext_sensor_io_rx.hpp"
#include "system/ext_sensor_io_tx.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_stop{false};

namespace {

int run_tx(const YAML::Node& root, int domain_id, const std::string& iface) {
    const auto uwb_node = root["uwb"];
    kist::ExtSensorIoTx::UwbSettings uwb;
    const bool uwb_enabled = uwb_node && uwb_node["enabled"].as<bool>(true);
    if (uwb_enabled) {
        uwb.serial_port = uwb_node["serial_port"].as<std::string>("/dev/uwb");
        uwb.serial_baud = uwb_node["serial_baud"].as<int>(115200);
        uwb.frame_id    = uwb_node["frame_id"].as<std::string>("map");
    }

    kist::ExtSensorIoTx tx;
    if (!tx.start(domain_id, iface, uwb_enabled ? &uwb : nullptr,
                  kist::cameras_from_yaml(root), kist::mics_from_yaml(root)))
        return 1;
    std::printf("[kist_ext_sensor_io] tx: %zu camera(s) + %zu mic(s)%s on domain=%d\n",
                tx.cameras().size(), tx.mics().size(),
                tx.uwb() ? " + uwb" : "", domain_id);

    struct Last { uint64_t c = 0, d = 0; };
    std::vector<Last> last(tx.cameras().size());
    std::vector<uint64_t> mic_last(tx.mics().size(), 0);
    auto last_fix_time = std::chrono::steady_clock::time_point{};
    uint64_t uwb_count = 0, uwb_last = 0;
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (auto* u = tx.uwb()) {
            auto s = u->samples().GetDataWithTime();
            if (s.HasData() && s.timestamp != last_fix_time) {
                last_fix_time = s.timestamp;
                uwb_count++;
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (size_t i = 0; i < tx.cameras().size(); ++i) {
            const auto& cam = tx.cameras()[i];
            const uint64_t c = cam.tx->color_published();
            const uint64_t d = cam.tx->depth_published();
            std::printf("  %-12s color %2llu fps  depth %2llu fps\n", cam.name.c_str(),
                        (unsigned long long)(c - last[i].c),
                        (unsigned long long)(d - last[i].d));
            last[i] = {c, d};
        }
        for (size_t i = 0; i < tx.mics().size(); ++i) {
            const auto& mic = tx.mics()[i];
            const uint64_t p = mic.tx->published();
            std::printf("  %-12s %llu chunk/s  overruns %llu\n", mic.name.c_str(),
                        (unsigned long long)(p - mic_last[i]),
                        (unsigned long long)mic.tx->overruns());
            mic_last[i] = p;
        }
        if (tx.uwb()) {
            std::printf("  %-12s %llu fix/s\n", "uwb",
                        (unsigned long long)(uwb_count - uwb_last));
            uwb_last = uwb_count;
        }
    }
    tx.stop();
    return 0;
}

int run_rx(const YAML::Node& root, int domain_id, const std::string& iface) {
    const auto uwb_node = root["uwb"];
    const bool uwb_enabled = uwb_node && uwb_node["enabled"].as<bool>(true);

    kist::ExtSensorIoRx rx;
    if (!rx.start(domain_id, iface, uwb_enabled,
                  kist::camera_names_from_yaml(root),
                  kist::mic_names_from_yaml(root)))
        return 1;
    std::printf("[kist_ext_sensor_io] rx: %zu camera(s) + %zu mic(s)%s on domain=%d\n",
                rx.cameras().size(), rx.mics().size(),
                rx.uwb() ? " + uwb" : "", domain_id);

    struct Last { uint64_t c = 0, d = 0; };
    std::vector<Last> last(rx.cameras().size());
    std::vector<uint64_t> mic_seen(rx.mics().size(), 0), mic_last(rx.mics().size(), 0);
    std::vector<std::chrono::steady_clock::time_point> mic_time(rx.mics().size());
    auto last_fix_time = std::chrono::steady_clock::time_point{};
    uint64_t uwb_count = 0, uwb_last = 0;
    auto window = std::chrono::steady_clock::now();
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (auto* u = rx.uwb()) {
            auto s = u->fix().GetDataWithTime();
            if (s.HasData() && s.timestamp != last_fix_time) {
                last_fix_time = s.timestamp;
                uwb_count++;
            }
        }
        for (size_t i = 0; i < rx.mics().size(); ++i) {
            auto c = rx.mics()[i].rx->chunk().GetDataWithTime();
            if (c.HasData() && c.timestamp != mic_time[i]) {
                mic_time[i] = c.timestamp;
                mic_seen[i]++;
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - window < std::chrono::seconds(1)) continue;
        window = now;
        for (size_t i = 0; i < rx.cameras().size(); ++i) {
            const auto& cam = rx.cameras()[i];
            const uint64_t c = cam.rx->color_decoded();
            const uint64_t d = cam.rx->depth_decoded();
            std::printf("  %-12s color %2llu fps  depth %2llu fps\n", cam.name.c_str(),
                        (unsigned long long)(c - last[i].c),
                        (unsigned long long)(d - last[i].d));
            last[i] = {c, d};
        }
        for (size_t i = 0; i < rx.mics().size(); ++i) {
            std::printf("  %-12s %llu chunk/s\n", rx.mics()[i].name.c_str(),
                        (unsigned long long)(mic_seen[i] - mic_last[i]));
            mic_last[i] = mic_seen[i];
        }
        if (rx.uwb()) {
            std::printf("  %-12s %llu fix/s\n", "uwb",
                        (unsigned long long)(uwb_count - uwb_last));
            uwb_last = uwb_count;
        }
    }
    rx.stop();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    if (argc < 2 || (std::strcmp(argv[1], "tx") != 0 && std::strcmp(argv[1], "rx") != 0)) {
        std::fprintf(stderr, "usage: %s <tx|rx> [config]\n", argv[0]);
        return 1;
    }
    const bool is_tx = std::strcmp(argv[1], "tx") == 0;
    const std::string config_path = (argc >= 3) ? argv[2] : "config/config.yaml";

    kist::Config::instance().load(config_path);
    const auto& root = kist::Config::instance().root();
    const int domain_id = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;
    const std::string iface;  // empty on purpose — the NIC comes from the DDS config XML

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    return is_tx ? run_tx(root, domain_id, iface) : run_rx(root, domain_id, iface);
}
