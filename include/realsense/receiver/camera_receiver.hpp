#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/receiver/color_decode_thread.hpp"
#include "realsense/receiver/depth_decode_thread.hpp"

#include <string>

namespace kist {

// Rx-side assembly (owns no thread of its own) — wires the camera Rx
// path on the consumer machine:
//   [ColorReceiver DDS cb] -> compressed buf -> [ColorDecodeThread R1] -> color()
//   [DepthReceiver DDS cb] -> compressed buf -> [DepthDecodeThread R2] -> depth()
// Consumers read the decoded buffers; empty = no live frame (1s watchdog
// on the underlying receivers).
class CameraReceiver {
public:
    bool start(int domain_id, const std::string& network_interface);
    void stop();

    DataBuffer<ColorFrame>& color() { return color_dec_.out; }
    DataBuffer<DepthFrame>& depth() { return depth_dec_.out; }

private:
    ColorDecodeThread color_dec_;
    DepthDecodeThread depth_dec_;
    bool              running_ = false;
};

} // namespace kist
