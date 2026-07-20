#include "system/ext_sensor_io_rx.hpp"

#include "common/config.hpp"

#include <iostream>

namespace kist {

ExtSensorIoRx& ExtSensorIoRx::instance() {
    static ExtSensorIoRx inst;
    return inst;
}

bool ExtSensorIoRx::start(const std::string& config_path) {
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const auto unitree_cfg = root["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    const auto interface   = unitree_cfg["network_interface"].as<std::string>();

    int started = 0;

    // ── uwb ─────────────────────────────────────────────────────
    if (const auto uwb = root["uwb"]; uwb && uwb["enabled"].as<bool>(true)) {
        if (!UwbReceiver::instance().start(domain_id, interface))
            return false;
        uwb_running_ = true;
        ++started;
    }

    // (camera, mic: one block each, later)

    if (started == 0) {
        std::cerr << "[ExtSensorIoRx] no sensor enabled in " << config_path << "\n";
        return false;
    }
    std::cout << "[ExtSensorIoRx] " << started << " receiver(s) running\n";
    return true;
}

void ExtSensorIoRx::stop() {
    if (uwb_running_) {
        UwbReceiver::instance().stop();
        uwb_running_ = false;
    }
}

} // namespace kist
