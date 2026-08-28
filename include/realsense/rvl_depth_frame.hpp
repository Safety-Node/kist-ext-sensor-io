#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// Compressed depth on the wire. The rt/ prefix keeps it ROS2-tool-visible
// as /kist/camera/depth/rvl; the codec suffix makes the topic self-
// describing. The payload is the idlc-generated kist_msgs::CompressedDepthFrame
// (RVL bitstream in its .data). Lives with the frame contract so Tx and
// Rx agree without either including the other's header (same as UWB's
// kUwbPoseTopic in uwb_position.hpp).
inline constexpr const char* kCameraDepthTopic = "rt/kist/camera/depth/rvl";

// Per-camera depth topic: rt/kist/camera/<name>/depth/rvl (see camera_color_topic).
inline std::string camera_depth_topic(const std::string& name) {
    return "rt/kist/camera/" + name + "/depth/rvl";
}

// RVL-compressed depth frame (Run-Length Variable-length, lossless) —
// the depth counterpart to the wire payload. Metadata mirrors the
// CompressedDepthFrame message; `data` is the RVL bitstream.
struct RvlDepthFrame {
    int           width       = 0;
    int           height      = 0;
    std::uint64_t sequence    = 0;
    std::int64_t  stamp_ns    = 0;
    std::string   frame_id;
    float         depth_scale = 0.001f;  // mm -> m

    // Pinhole intrinsics of this frame (px), carried through to the consumer.
    float         fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;

    std::vector<std::uint8_t> data;  // RVL encoded

    bool empty() const { return data.empty(); }
};

} // namespace kist
