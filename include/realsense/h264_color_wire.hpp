#pragma once

#include "realsense/h264_color_frame.hpp"

#include <cstdint>
#include <vector>

namespace kist {

// Wire (de)serialization for a compressed color frame — the payload that
// travels in VoxelMapCompressed_.data over DDS. Pure Module; the color
// counterpart to serialize_depth. serialize->deserialize is byte-exact.
std::vector<uint8_t> serialize_color(const H264ColorFrame& frame);
H264ColorFrame       deserialize_color(const std::vector<uint8_t>& blob);

} // namespace kist
