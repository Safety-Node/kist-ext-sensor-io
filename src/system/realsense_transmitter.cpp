#include "system/realsense_transmitter.hpp"

#include "realsense/h264_color_frame.hpp"   // camera_color_topic()
#include "realsense/rvl_depth_frame.hpp"    // camera_depth_topic()

#include <iostream>

namespace kist {

bool RealsenseTransmitter::start(int domain_id, const std::string& network_interface,
                                 const std::string& name,
                                 const CameraCaptureConfig& capture_cfg,
                                 const H264EncoderConfig& color_cfg) {
    if (running_) return true;

    const std::string color_topic = camera_color_topic(name);
    const std::string depth_topic = camera_depth_topic(name);

    if (!capture_.start(capture_cfg))
        return false;
    if (!depth_.start(domain_id, network_interface, capture_.depth_buf, depth_topic)) {
        capture_.stop();
        return false;
    }
    if (capture_cfg.color_enabled &&
        !color_.start(domain_id, network_interface, capture_.color_buf, color_cfg, color_topic)) {
        depth_.stop();
        capture_.stop();
        return false;
    }

    running_ = true;
    std::cout << "[RealsenseTransmitter] '" << name << "' running (color="
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
