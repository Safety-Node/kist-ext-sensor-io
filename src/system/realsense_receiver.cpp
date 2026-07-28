#include "system/realsense_receiver.hpp"

#include "realsense/h264_color_frame.hpp"   // camera_color_topic()
#include "realsense/rvl_depth_frame.hpp"    // camera_depth_topic()

#include <iostream>

namespace kist {

bool RealsenseReceiver::start(int domain_id, const std::string& network_interface,
                              const std::string& name) {
    if (running_) return true;

    if (!color_sub_.start(domain_id, network_interface, camera_color_topic(name)))
        return false;
    if (!depth_sub_.start(domain_id, network_interface, camera_depth_topic(name))) {
        color_sub_.stop();
        return false;
    }

    color_dec_.start(color_sub_.color_buf);
    depth_dec_.start(depth_sub_.depth_buf);
    running_ = true;
    std::cout << "[RealsenseReceiver] '" << name << "' running (color + depth decode)\n";
    return true;
}

void RealsenseReceiver::stop() {
    if (!running_) return;
    // Tear down downstream-first: decode threads before the DDS subscribers.
    color_dec_.stop();
    depth_dec_.stop();
    color_sub_.stop();
    depth_sub_.stop();
    running_ = false;
}

} // namespace kist
