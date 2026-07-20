#include "realsense/receiver/camera_receiver.hpp"

#include "realsense/receiver/color_receiver.hpp"
#include "realsense/receiver/depth_receiver.hpp"

#include <iostream>

namespace kist {

bool CameraReceiver::start(int domain_id, const std::string& network_interface) {
    if (running_) return true;

    auto& crx = ColorReceiver::instance();
    auto& drx = DepthReceiver::instance();
    if (!crx.start(domain_id, network_interface) ||
        !drx.start(domain_id, network_interface))
        return false;

    color_dec_.start(crx.color_buf);
    depth_dec_.start(drx.depth_buf);
    running_ = true;
    std::cout << "[CameraReceiver] running (color + depth decode)\n";
    return true;
}

void CameraReceiver::stop() {
    if (!running_) return;
    color_dec_.stop();
    depth_dec_.stop();
    ColorReceiver::instance().stop();
    DepthReceiver::instance().stop();
    running_ = false;
}

} // namespace kist
