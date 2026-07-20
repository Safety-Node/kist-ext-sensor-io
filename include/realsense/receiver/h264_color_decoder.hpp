#pragma once

#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"

#include <memory>
#include <optional>

namespace kist {

// H.264 color decoder — Rx side, libavcodec. Pure Module (no thread, no
// DDS): Annex-B NAL units -> BGR8 raw (via libswscale). Ported from
// depth-cam-stream-codec. Returns nullopt while the decoder is still
// buffering (e.g. before the first keyframe).
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

} // namespace kist
