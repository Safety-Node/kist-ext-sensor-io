#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// Raw color frame as captured (BGR8). Ported from depth-cam-stream-codec
// (our own repo) — the codec cores are reused, only the ROS2 transport
// is replaced by the DDS Transmitter/Receiver pattern.
struct ColorFrame {
    int width        = 0;
    int height       = 0;
    int stride_bytes = 0;

    std::uint64_t sequence = 0;
    std::int64_t  stamp_ns = 0;
    std::string   frame_id;

    std::vector<std::uint8_t> data;  // BGR8

    bool empty() const { return data.empty(); }
};

} // namespace kist
