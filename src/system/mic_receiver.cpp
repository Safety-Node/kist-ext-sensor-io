#include "system/mic_receiver.hpp"

namespace kist {

bool MicReceiver::start(int domain_id, const std::string& network_interface,
                        const std::string& name) {
    if (running_) return true;
    if (!sub_.start(domain_id, network_interface, mic_audio_topic(name)))
        return false;
    running_ = true;
    return true;
}

void MicReceiver::stop() {
    sub_.stop();
    running_ = false;
}

} // namespace kist
