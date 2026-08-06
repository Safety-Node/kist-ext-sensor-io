#include "system/ext_sensor_io_tx.hpp"

#include <iostream>

namespace kist {

bool ExtSensorIoTx::start(int domain_id, const std::string& network_interface,
                          const UwbSettings* uwb,
                          const std::vector<CameraSpec>& cameras,
                          const std::vector<MicSpec>& mics) {
    if (running_) return true;

    if (uwb) {
        uwb_ = std::make_unique<UwbTransmitter>();
        if (!uwb_->start(domain_id, network_interface, uwb->serial_port,
                         uwb->serial_baud, uwb->frame_id)) {
            std::cerr << "[ExtSensorIoTx] UWB failed to start — skipped\n";
            uwb_.reset();
        }
    }

    for (const auto& spec : cameras) {
        auto tx = std::make_unique<RealsenseTransmitter>();
        if (!tx->start(domain_id, network_interface, spec.name,
                       spec.capture, spec.encoder)) {
            std::cerr << "[ExtSensorIoTx] camera '" << spec.name
                      << "' failed to start — skipped\n";
            continue;   // one bad camera shouldn't take down the others
        }
        cams_.push_back(Camera{spec.name, std::move(tx)});
    }

    for (const auto& spec : mics) {
        auto tx = std::make_unique<MicTransmitter>();
        if (!tx->start(domain_id, network_interface, spec.name, spec.capture)) {
            std::cerr << "[ExtSensorIoTx] mic '" << spec.name
                      << "' failed to start — skipped\n";
            continue;
        }
        mics_.push_back(Mic{spec.name, std::move(tx)});
    }

    if (cams_.empty() && mics_.empty() && !uwb_) {
        std::cerr << "[ExtSensorIoTx] nothing started\n";
        return false;
    }
    running_ = true;
    return true;
}

void ExtSensorIoTx::stop() {
    for (auto& cam : cams_)
        cam.tx->stop();
    cams_.clear();
    for (auto& mic : mics_)
        mic.tx->stop();
    mics_.clear();
    if (uwb_) {
        uwb_->stop();
        uwb_.reset();
    }
    running_ = false;
}

} // namespace kist
