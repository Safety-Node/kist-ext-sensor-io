#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

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

    std::vector<std::uint8_t> data;  // RVL encoded

    bool empty() const { return data.empty(); }
};

} // namespace kist
