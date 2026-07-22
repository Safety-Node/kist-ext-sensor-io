#include "system/realsense_transmitter.hpp"

#include <iostream>

namespace kist {

bool RealsenseTransmitter::start(int domain_id, const std::string& network_interface,
                                 const CameraCaptureConfig& capture_cfg,
                                 const H264EncoderConfig& color_cfg) {
    if (running_) return true;

    if (!capture_.start(capture_cfg))
        return false;
    if (!depth_.start(domain_id, network_interface, capture_.depth_buf)) {
        capture_.stop();
        return false;
    }
    if (capture_cfg.color_enabled &&
        !color_.start(domain_id, network_interface, capture_.color_buf, color_cfg)) {
        depth_.stop();
        capture_.stop();
        return false;
    }

    running_ = true;
    std::cout << "[RealsenseTransmitter] running (color="
              << (capture_cfg.color_enabled ? "on" : "off") << ", depth=on)\n";
    return true;
}

void RealsenseTransmitter::stop() {
    if (!running_) return;
    // Tear down downstream-first: encode-publishers before capture.
    color_.stop();
    depth_.stop();
    capture_.stop();
    running_ = false;
}

} // namespace kist
