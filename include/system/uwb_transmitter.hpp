#pragma once

#include "uwb/transmitter/uwb_publisher.hpp"
#include "uwb/transmitter/uwb_serial_reader.hpp"

#include <string>

namespace kist {

// UWB Tx assembly (the mirror of UwbReceiver in system/) — wires the two
// Tx threads, one class per thread:
//
//   [UwbSerialReader]  /dev/uwb -> parse -> sample_buf
//   [UwbPublisher]     sample_buf -> DDS rt/kist/uwb/pose
//
// Runs on the machine the dongle is plugged into (the G1 onboard PC).
// Owns no thread itself; start() wires reader -> publisher and stop()
// tears them down in reverse.
class UwbTransmitter {
public:
    bool start(int domain_id, const std::string& network_interface,
               const std::string& serial_port, int serial_baud,
               const std::string& frame_id,
               const std::string& topic = kUwbPoseTopic);
    void stop();

private:
    UwbSerialReader serial_;
    UwbPublisher    publisher_;
};

} // namespace kist
