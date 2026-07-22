#include "system/realsense_receiver.hpp"

#include <iostream>

namespace kist {

bool RealsenseReceiver::start(int domain_id, const std::string& network_interface) {
    if (running_) return true;

    if (!color_sub_.start(domain_id, network_interface))
        return false;
    if (!depth_sub_.start(domain_id, network_interface)) {
        color_sub_.stop();
        return false;
    }

    color_dec_.start(color_sub_.color_buf);
    depth_dec_.start(depth_sub_.depth_buf);
    running_ = true;
    std::cout << "[RealsenseReceiver] running (color + depth decode)\n";
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
