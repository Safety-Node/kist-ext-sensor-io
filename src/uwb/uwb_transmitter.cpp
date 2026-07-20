#include "uwb/uwb_transmitter.hpp"

#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <iostream>

namespace kist {

UwbTransmitter::UwbTransmitter() = default;
UwbTransmitter::~UwbTransmitter() = default;

bool UwbTransmitter::start(int domain_id, const std::string& network_interface,
                         const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        pub_.reset(new PosePub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[UwbTransmitter] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[UwbTransmitter] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void UwbTransmitter::publish(const UwbSample& sample, const std::string& frame_id) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    geometry_msgs::msg::dds_::PoseStamped_ msg;
    msg.header().stamp().sec(int32_t(ns / 1000000000LL));
    msg.header().stamp().nanosec(uint32_t(ns % 1000000000LL));
    msg.header().frame_id(frame_id);
    msg.pose().position().x(sample.x);
    msg.pose().position().y(sample.y);
    msg.pose().position().z(sample.z);
    msg.pose().orientation().x(0.0);
    msg.pose().orientation().y(0.0);
    msg.pose().orientation().z(0.0);
    msg.pose().orientation().w(1.0);  // no heading source

    pub_->Write(msg);
}

} // namespace kist
