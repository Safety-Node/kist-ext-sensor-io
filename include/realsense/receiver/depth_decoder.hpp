#pragma once

#include "common/data_buffer.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/rvl_depth_frame.hpp"

#include <atomic>
#include <cstdint>
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

    // Monotonic count of frames successfully decoded into `out`. Poll the delta
    // over a window for a produce-site-accurate decode rate (throughput / health).
    uint64_t decoded() const { return decoded_.load(std::memory_order_relaxed); }

private:
    void run();

    DataBuffer<RvlDepthFrame>* source_ = nullptr;
    RvlDepthDecoder            decoder_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};
    std::atomic<uint64_t>      decoded_{0};
};

} // namespace kist
