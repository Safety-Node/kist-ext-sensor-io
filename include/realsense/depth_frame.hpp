#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// Raw depth frame as captured (Z16: 16-bit depth per pixel). Ported from
// depth-cam-stream-codec.
struct DepthFrame {
    int width        = 0;
    int height       = 0;
    int stride_bytes = 0;

    std::uint64_t sequence    = 0;
    std::int64_t  stamp_ns    = 0;
    std::string   frame_id;
    float         depth_scale = 0.001f;  // meters per Z16 unit

    std::vector<std::uint8_t> data;  // Z16

    bool empty() const { return data.empty(); }
};

} // namespace kist
