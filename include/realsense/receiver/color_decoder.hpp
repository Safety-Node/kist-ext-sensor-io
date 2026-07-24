#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"

#include <atomic>
#include <memory>
#include <optional>
#include <thread>

namespace kist {

// H.264 color decoder — Rx side, libavcodec. Pure Module (no thread, no
// DDS): Annex-B NAL units -> BGR8 raw (via libswscale). Returns nullopt
// while the decoder is still buffering (e.g. before the first keyframe).
// Stateful (libav contexts persist across frames), so it lives as an owned
// object of the decode thread below rather than inlined into its loop.
class H264ColorDecoder {
public:
    H264ColorDecoder();
    ~H264ColorDecoder();

    H264ColorDecoder(const H264ColorDecoder&)            = delete;
    H264ColorDecoder& operator=(const H264ColorDecoder&) = delete;

    std::optional<ColorFrame> decode(const H264ColorFrame& frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

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
