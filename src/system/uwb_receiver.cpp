#include "system/uwb_receiver.hpp"

#include <iostream>

namespace kist {

bool UwbReceiver::start(int domain_id, const std::string& network_interface) {
    if (running_) return true;
    if (!sub_.start(domain_id, network_interface))
        return false;
    running_ = true;
    std::cout << "[UwbReceiver] running\n";
    return true;
}

void UwbReceiver::stop() {
    if (!running_) return;
    sub_.stop();
    running_ = false;
}

} // namespace kist
