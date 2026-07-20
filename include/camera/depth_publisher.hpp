#pragma once

#include "camera/rvl_depth_frame.hpp"

#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace unitree_go::msg::dds_ {
class VoxelMapCompressed_;
}

namespace kist {

// Compressed depth on the wire. rt/ prefix keeps it ROS2-tool-visible as
// /kist/camera/depth; the payload rides VoxelMapCompressed_.data (a
// vendored byte-carrier IDL type — no custom IDL / idlc needed).
inline constexpr const char* kCameraDepthTopic = "rt/kist/camera/depth";

// Publishes an RVL-compressed depth frame over DDS. Serializes the frame
// to bytes (rvl_depth_wire) and writes them in a VoxelMapCompressed_.
// Thread-less on its own — a DepthEncodePublisher thread will call
// publish() per frame (slice 4); for now it is driven directly.
class DepthPublisher {
public:
    DepthPublisher();
    ~DepthPublisher();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kCameraDepthTopic);
    void publish(const RvlDepthFrame& frame);

private:
    using Pub = unitree::robot::ChannelPublisher<unitree_go::msg::dds_::VoxelMapCompressed_>;
    std::unique_ptr<Pub> pub_;
};

} // namespace kist
