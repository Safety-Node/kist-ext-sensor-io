#pragma once

#include "camera/rvl_depth_frame.hpp"

#include <cstdint>
#include <vector>

namespace kist {

// Wire (de)serialization for a compressed depth frame — the payload that
// travels in VoxelMapCompressed_.data over DDS. Pure Module: no thread,
// no DDS; serialize->deserialize is a byte-exact round-trip.
std::vector<uint8_t> serialize_depth(const RvlDepthFrame& frame);
RvlDepthFrame        deserialize_depth(const std::vector<uint8_t>& blob);

} // namespace kist
