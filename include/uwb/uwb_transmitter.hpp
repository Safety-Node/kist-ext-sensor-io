#pragma once

#include "uwb/uwb_publisher.hpp"
#include "uwb/uwb_serial_reader.hpp"

#include <string>

namespace kist {

// Tx half, self-contained (the mirror of UwbReceiver) — the assembly of
// the two Tx threads, one class per thread:
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
