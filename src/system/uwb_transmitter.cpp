#include "system/uwb_transmitter.hpp"

namespace kist {

bool UwbTransmitter::start(int domain_id, const std::string& network_interface,
                           const std::string& serial_port, int serial_baud,
                           const std::string& frame_id,
                           const std::string& topic) {
    if (!publisher_.start(domain_id, network_interface,
                          serial_.sample_buf, frame_id, topic))
        return false;
    serial_.start(serial_port, serial_baud);
    return true;
}

void UwbTransmitter::stop() {
    serial_.stop();
    publisher_.stop();
}

} // namespace kist
