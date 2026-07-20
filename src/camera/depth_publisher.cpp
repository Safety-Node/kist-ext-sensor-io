#include "camera/depth_publisher.hpp"

#include "camera/rvl_depth_wire.hpp"

#include <unitree/idl/go2/VoxelMapCompressed_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <iostream>

namespace kist {

DepthPublisher::DepthPublisher() = default;
DepthPublisher::~DepthPublisher() = default;

bool DepthPublisher::start(int domain_id, const std::string& network_interface,
                           const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[DepthPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    std::cout << "[DepthPublisher] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void DepthPublisher::publish(const RvlDepthFrame& frame) {
    unitree_go::msg::dds_::VoxelMapCompressed_ msg;
    // Carrier used purely as a byte bag; our schema lives in the blob.
    msg.data() = serialize_depth(frame);
    pub_->Write(msg);
}

} // namespace kist
