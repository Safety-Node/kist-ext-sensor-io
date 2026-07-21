#pragma once

#include "realsense/depth_frame.hpp"
#include "realsense/rvl_depth_frame.hpp"

namespace kist {

// RVL (Run-Length Variable-length) lossless depth encoder — Tx side.
// Pure Module (no thread, no DDS): Z16 raw -> RVL bitstream. Ported
// verbatim from depth-cam-stream-codec.
class RvlEncoder {
public:
    RvlDepthFrame encode(const DepthFrame& frame) const;
};

} // namespace kist
