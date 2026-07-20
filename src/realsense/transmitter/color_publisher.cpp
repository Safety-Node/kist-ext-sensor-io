#include "realsense/transmitter/color_publisher.hpp"

#include "realsense/h264_color_wire.hpp"

#include <unitree/idl/go2/VoxelMapCompressed_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <iostream>

namespace kist {

ColorPublisher::ColorPublisher() = default;
ColorPublisher::~ColorPublisher() = default;

bool ColorPublisher::start(int domain_id, const std::string& network_interface,
                           const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[ColorPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[ColorPublisher] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void ColorPublisher::publish(const H264ColorFrame& frame) {
    unitree_go::msg::dds_::VoxelMapCompressed_ msg;
    msg.data() = serialize_color(frame);
    pub_->Write(msg);
}

} // namespace kist
