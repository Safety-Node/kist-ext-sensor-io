#pragma once

#include "camera/depth_frame.hpp"
#include "camera/rvl_depth_frame.hpp"

namespace kist {

// RVL (Run-Length Variable-length) lossless depth codec, ported verbatim
// from depth-cam-stream-codec — a pure algorithm, no threads, no
// transport. encode(): Z16 raw -> RVL bitstream; decode(): the inverse
// (a lossless round-trip).

class RvlDepthEncoder {
public:
    RvlDepthFrame encode(const DepthFrame& frame) const;
};

class RvlDepthDecoder {
public:
    DepthFrame decode(const RvlDepthFrame& frame) const;
};

} // namespace kist
