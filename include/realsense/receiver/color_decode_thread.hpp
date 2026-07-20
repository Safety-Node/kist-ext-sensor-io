#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"
#include "realsense/receiver/h264_color_decoder.hpp"

#include <atomic>
#include <memory>
#include <thread>

namespace kist {

// Rx thread (R1): polls a compressed-color buffer, H.264-decodes new
// frames, and publishes raw BGR to `out`. Decode is heavy, so it runs
// off the DDS receive callback (which only deserializes). `source` must
// outlive this object.
class ColorDecodeThread {
public:
    bool start(DataBuffer<H264ColorFrame>& source);
    void stop();

    DataBuffer<ColorFrame> out;  // decoded BGR

private:
    void run();

    DataBuffer<H264ColorFrame>*       source_ = nullptr;
    std::unique_ptr<H264ColorDecoder> decoder_;
    std::thread                       thread_;
    std::atomic<bool>                 running_{false};
};

} // namespace kist
