#pragma once

#include "uwb/dwm_serial.hpp"
#include "uwb/uwb_position.hpp"

#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

// Tx half: publishes valid DWM fixes as PoseStamped over DDS (no ROS2).
// Runs on the machine the dongle is plugged into (the G1 onboard PC).
// Orientation is identity — UWB has no heading source.
class UwbTransmitter {
public:
    UwbTransmitter();
    ~UwbTransmitter();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kUwbPoseTopic);

    // Stamps with the system clock and writes to DDS.
    void publish(const UwbSample& sample, const std::string& frame_id);

private:
    using PosePub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<PosePub> pub_;
};

} // namespace kist
