#pragma once

#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"

#include <cstdint>
#include <optional>
#include <string>

struct x264_t;                 // opaque, from <x264.h> in the .cpp
struct x264_picture_t;

namespace kist {

struct H264EncoderConfig {
    int         width             = 640;
    int         height            = 480;
    int         fps               = 30;
    int         bitrate_kbps      = 4000;
    std::string preset            = "veryfast";
    std::string tune              = "zerolatency";
    std::string profile           = "baseline";
    int         keyframe_interval = 30;   // fixed GOP
};

// H.264 color encoder — Tx side, x264. Pure Module (no thread, no DDS):
// BGR8 raw -> Annex-B NAL units. Ported from depth-cam-stream-codec.
// Lossy, unlike the RVL depth codec.
class H264Encoder {
public:
    explicit H264Encoder(const H264EncoderConfig& cfg);
    ~H264Encoder();

    H264Encoder(const H264Encoder&)            = delete;
    H264Encoder& operator=(const H264Encoder&) = delete;

    std::optional<H264ColorFrame> encode(const ColorFrame& frame);

private:
    void open(const H264EncoderConfig& cfg);
    void close();
    void reset(const H264EncoderConfig& cfg);
    void bgr8_to_i420(const uint8_t* bgr, int width, int height, int stride_bgr);

    H264EncoderConfig cfg_;
    x264_t*           encoder_ = nullptr;
    x264_picture_t*   pic_in_  = nullptr;   // heap so the header stays x264-free
    int64_t           frame_count_ = 0;
};

} // namespace kist
