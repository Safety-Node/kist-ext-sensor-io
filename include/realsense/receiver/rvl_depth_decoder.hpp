#pragma once

#include "realsense/depth_frame.hpp"
#include "realsense/rvl_depth_frame.hpp"

namespace kist {

// RVL lossless depth decoder — Rx side. Pure Module (no thread, no DDS):
// RVL bitstream -> Z16 raw, the exact inverse of RvlDepthEncoder.
class RvlDepthDecoder {
public:
    DepthFrame decode(const RvlDepthFrame& frame) const;
};

} // namespace kist
