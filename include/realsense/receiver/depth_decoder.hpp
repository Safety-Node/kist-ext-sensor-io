#pragma once

#include "common/data_buffer.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/rvl_depth_frame.hpp"

#include <atomic>
#include <thread>

namespace kist {

// RVL lossless depth decoder — Rx side. Pure Module (no thread, no DDS):
// RVL bitstream -> Z16 raw, the exact inverse of RvlEncoder. Stateless, so
// it's a plain owned member of the decode thread below.
class RvlDepthDecoder {
public:
    DepthFrame decode(const RvlDepthFrame& frame) const;
};

// Rx thread (R2): polls a compressed-depth buffer, RVL-decodes new
// frames, and publishes raw Z16 to `out`. Runs off the DDS receive
// callback. `source` must outlive this object.
class DepthDecodeThread {
public:
    bool start(DataBuffer<RvlDepthFrame>& source);
    void stop();

    DataBuffer<DepthFrame> out;  // decoded Z16

private:
    void run();

    DataBuffer<RvlDepthFrame>* source_ = nullptr;
    RvlDepthDecoder            decoder_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};
};

} // namespace kist
