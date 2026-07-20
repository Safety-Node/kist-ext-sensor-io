#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// Compressed color on the wire. rt/ prefix keeps it ROS2-tool-visible as
// /kist/camera/color; the payload rides VoxelMapCompressed_.data. Lives
// with the frame contract so Tx and Rx agree without either including
// the other's header (same pattern as kCameraDepthTopic).
inline constexpr const char* kCameraColorTopic = "rt/kist/camera/color";

// H.264-compressed color frame (Annex-B NAL units) — the color
// counterpart to RvlDepthFrame. `is_keyframe` lets a late-joining
// receiver know when it can start decoding.
struct H264ColorFrame {
    int           width       = 0;
    int           height      = 0;
    std::uint64_t sequence    = 0;
    std::int64_t  stamp_ns    = 0;
    std::string   frame_id;
    bool          is_keyframe = false;

    std::vector<std::uint8_t> data;  // Annex-B H.264 NAL units

    bool empty() const { return data.empty(); }
};

} // namespace kist
