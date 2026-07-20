#pragma once

#include "realsense/h264_color_frame.hpp"

#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace unitree_go::msg::dds_ {
class VoxelMapCompressed_;
}

namespace kist {

// Publishes an H.264-compressed color frame over DDS — the color twin of
// DepthPublisher. Serializes to bytes (h264_color_wire) into a
// VoxelMapCompressed_. Takes an already-encoded frame; the encode thread
// calls publish() per frame.
class ColorPublisher {
public:
    ColorPublisher();
    ~ColorPublisher();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kCameraColorTopic);
    void publish(const H264ColorFrame& frame);

private:
    using Pub = unitree::robot::ChannelPublisher<unitree_go::msg::dds_::VoxelMapCompressed_>;
    std::unique_ptr<Pub> pub_;
};

} // namespace kist
