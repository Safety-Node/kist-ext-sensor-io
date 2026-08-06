#include "system/ext_sensor_io_rx.hpp"

#include <iostream>

namespace kist {

bool ExtSensorIoRx::start(int domain_id, const std::string& network_interface,
                          bool uwb_enabled,
                          const std::vector<std::string>& camera_names,
                          const std::vector<std::string>& mic_names) {
    if (running_) return true;

    if (uwb_enabled) {
        uwb_ = std::make_unique<UwbReceiver>();
        if (!uwb_->start(domain_id, network_interface)) {
            std::cerr << "[ExtSensorIoRx] UWB failed to start — skipped\n";
            uwb_.reset();
        }
    }

    for (const auto& name : camera_names) {
        auto rx = std::make_unique<RealsenseReceiver>();
        if (!rx->start(domain_id, network_interface, name)) {
            std::cerr << "[ExtSensorIoRx] camera '" << name
                      << "' failed to start — skipped\n";
            continue;
        }
        cams_.push_back(Camera{name, std::move(rx)});
    }

    for (const auto& name : mic_names) {
        auto rx = std::make_unique<MicReceiver>();
        if (!rx->start(domain_id, network_interface, name)) {
            std::cerr << "[ExtSensorIoRx] mic '" << name
                      << "' failed to start — skipped\n";
            continue;
        }
        mics_.push_back(Mic{name, std::move(rx)});
    }

    if (cams_.empty() && mics_.empty() && !uwb_) {
        std::cerr << "[ExtSensorIoRx] nothing started\n";
        return false;
    }
    running_ = true;
    return true;
}

void ExtSensorIoRx::stop() {
    for (auto& cam : cams_)
        cam.rx->stop();
    cams_.clear();
    for (auto& mic : mics_)
        mic.rx->stop();
    mics_.clear();
    if (uwb_) {
        uwb_->stop();
        uwb_.reset();
    }
    running_ = false;
}

} // namespace kist
