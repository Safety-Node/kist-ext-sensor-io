#include "system/ext_sensor_io_tx.hpp"

#include "common/config.hpp"

#include <iostream>

namespace kist {

ExtSensorIoTx& ExtSensorIoTx::instance() {
    static ExtSensorIoTx inst;
    return inst;
}

bool ExtSensorIoTx::start(const std::string& config_path) {
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const auto unitree_cfg = root["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    const auto interface   = unitree_cfg["network_interface"].as<std::string>();

    int started = 0;

    // ── uwb ─────────────────────────────────────────────────────
    if (const auto uwb = root["uwb"]; uwb && uwb["enabled"].as<bool>(true)) {
        if (!uwb_.start(domain_id, interface,
                        uwb["serial_port"].as<std::string>(),
                        uwb["serial_baud"].as<int>(),
                        uwb["frame_id"].as<std::string>()))
            return false;
        uwb_running_ = true;
        ++started;
    }

    // (camera, mic: one block each, later)

    if (started == 0) {
        std::cerr << "[ExtSensorIoTx] no sensor enabled in " << config_path << "\n";
        return false;
    }
    std::cout << "[ExtSensorIoTx] " << started << " transmitter(s) running\n";
    return true;
}

void ExtSensorIoTx::stop() {
    if (uwb_running_) {
        uwb_.stop();
        uwb_running_ = false;
    }
}

} // namespace kist
