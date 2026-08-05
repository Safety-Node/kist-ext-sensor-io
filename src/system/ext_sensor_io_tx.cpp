#include "system/ext_sensor_io_tx.hpp"

#include <iostream>

namespace kist {

bool ExtSensorIoTx::start(int domain_id, const std::string& network_interface,
                          const UwbSettings* uwb,
                          const std::vector<CameraSpec>& cameras) {
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

    if (cams_.empty() && !uwb_) {
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
    if (uwb_) {
        uwb_->stop();
        uwb_.reset();
    }
    running_ = false;
}

} // namespace kist
