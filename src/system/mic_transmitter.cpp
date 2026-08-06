#include "system/mic_transmitter.hpp"

namespace kist {

bool MicTransmitter::start(int domain_id, const std::string& network_interface,
                           const std::string& name, const MicCaptureConfig& cfg) {
    if (running_) return true;
    if (!pub_.start(domain_id, network_interface, cfg, mic_audio_topic(name)))
        return false;
    running_ = true;
    return true;
}

void MicTransmitter::stop() {
    pub_.stop();
    running_ = false;
}

} // namespace kist
