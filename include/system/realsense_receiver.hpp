#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/receiver/color_subscriber.hpp"
#include "realsense/receiver/depth_subscriber.hpp"
#include "realsense/receiver/color_decoder.hpp"
#include "realsense/receiver/depth_decoder.hpp"

#include <string>

namespace kist {

// RealSense camera Rx assembly (owns no thread of its own) — wires the
// subscribers + decode threads and manages their lifecycle:
//   [ColorSubscriber] -> compressed buf -> [ColorDecodeThread] -> color()
//   [DepthSubscriber] -> compressed buf -> [DepthDecodeThread] -> depth()
// Embed it in a consumer app (segmentation/fusion) and read the decoded
// frames in-process via color()/depth(); or run it via the
// test_realsense_receiver executable. Empty buffer = no live frame (1s
// watchdog on the subscribers).
class RealsenseReceiver {
public:
    bool start(int domain_id, const std::string& network_interface);
    void stop();

    DataBuffer<ColorFrame>& color() { return color_dec_.out; }
    DataBuffer<DepthFrame>& depth() { return depth_dec_.out; }

private:
    ColorSubscriber   color_sub_;
    DepthSubscriber   depth_sub_;
    ColorDecodeThread color_dec_;
    DepthDecodeThread depth_dec_;
    bool              running_ = false;
};

} // namespace kist
