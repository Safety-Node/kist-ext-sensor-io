#include "realsense/transmitter/h264_encoder.hpp"

#include <x264.h>

#include <stdexcept>

namespace kist {

H264Encoder::H264Encoder(const H264EncoderConfig& cfg) {
    pic_in_ = new x264_picture_t{};
    open(cfg);
}

H264Encoder::~H264Encoder() {
    close();
    delete pic_in_;
}

void H264Encoder::open(const H264EncoderConfig& cfg) {
    cfg_ = cfg;

    x264_param_t param;
    if (x264_param_default_preset(&param, cfg.preset.c_str(), cfg.tune.c_str()) < 0)
        throw std::runtime_error("x264_param_default_preset failed");

    param.i_log_level          = X264_LOG_NONE;
    param.i_csp                = X264_CSP_I420;
    param.i_width              = cfg.width;
    param.i_height             = cfg.height;
    param.i_fps_num            = static_cast<uint32_t>(cfg.fps);
    param.i_fps_den            = 1;
    param.i_keyint_max         = cfg.keyframe_interval;
    param.i_keyint_min         = cfg.keyframe_interval;  // fixed GOP
    param.i_scenecut_threshold = 0;                      // no scene-cut IDR
    param.i_bframe             = 0;                       // no B-frames
    param.b_repeat_headers     = 1;                       // SPS/PPS before each keyframe
    param.b_annexb             = 1;
    param.rc.i_rc_method       = X264_RC_ABR;
    param.rc.i_bitrate         = cfg.bitrate_kbps;

    if (x264_param_apply_profile(&param, cfg.profile.c_str()) < 0)
        throw std::runtime_error("x264_param_apply_profile failed");

    encoder_ = x264_encoder_open(&param);
    if (!encoder_)
        throw std::runtime_error("x264_encoder_open failed");

    if (x264_picture_alloc(pic_in_, X264_CSP_I420, cfg.width, cfg.height) < 0) {
        x264_encoder_close(encoder_);
        encoder_ = nullptr;
        throw std::runtime_error("x264_picture_alloc failed");
    }
    frame_count_ = 0;
}

void H264Encoder::close() {
    if (encoder_) {
        x264_picture_clean(pic_in_);
        x264_encoder_close(encoder_);
        encoder_ = nullptr;
    }
}

void H264Encoder::reset(const H264EncoderConfig& cfg) {
    close();
    open(cfg);
}

std::optional<H264ColorFrame> H264Encoder::encode(const ColorFrame& frame) {
    if (!encoder_) return std::nullopt;

    if (frame.width != cfg_.width || frame.height != cfg_.height) {
        H264EncoderConfig c = cfg_;
        c.width = frame.width;
        c.height = frame.height;
        reset(c);
    }

    bgr8_to_i420(frame.data.data(), frame.width, frame.height, frame.stride_bytes);
    pic_in_->i_pts = frame_count_++;

    x264_nal_t*    nals;
    int            nal_count;
    x264_picture_t pic_out{};
    const int frame_size = x264_encoder_encode(encoder_, &nals, &nal_count, pic_in_, &pic_out);
    if (frame_size <= 0) return std::nullopt;

    H264ColorFrame result;
    result.width       = frame.width;
    result.height      = frame.height;
    result.sequence    = frame.sequence;
    result.stamp_ns    = frame.stamp_ns;
    result.frame_id    = frame.frame_id;
    result.is_keyframe = (pic_out.b_keyframe != 0);
    result.data.reserve(static_cast<size_t>(frame_size));
    for (int i = 0; i < nal_count; ++i) {
        const uint8_t* p = nals[i].p_payload;
        result.data.insert(result.data.end(), p, p + nals[i].i_payload);
    }
    return result;
}

void H264Encoder::bgr8_to_i420(const uint8_t* bgr, int width, int height, int stride_bgr) {
    uint8_t*  y_plane  = pic_in_->img.plane[0];
    uint8_t*  u_plane  = pic_in_->img.plane[1];
    uint8_t*  v_plane  = pic_in_->img.plane[2];
    const int y_stride = pic_in_->img.i_stride[0];
    const int u_stride = pic_in_->img.i_stride[1];
    const int v_stride = pic_in_->img.i_stride[2];

    for (int row = 0; row < height; ++row) {
        const uint8_t* src   = bgr + row * stride_bgr;
        uint8_t*       y_dst = y_plane + row * y_stride;

        for (int col = 0; col < width; ++col) {
            const int b = src[col * 3 + 0];
            const int g = src[col * 3 + 1];
            const int r = src[col * 3 + 2];
            y_dst[col] = static_cast<uint8_t>(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
        }

        if (row % 2 == 0) {
            const uint8_t* src0 = src;
            const uint8_t* src1 = (row + 1 < height) ? bgr + (row + 1) * stride_bgr : src;
            uint8_t* u_dst = u_plane + (row / 2) * u_stride;
            uint8_t* v_dst = v_plane + (row / 2) * v_stride;

            for (int col = 0; col < width; col += 2) {
                const int r00 = src0[ col      * 3 + 2], g00 = src0[ col      * 3 + 1], b00 = src0[ col      * 3 + 0];
                const int r01 = src0[(col + 1) * 3 + 2], g01 = src0[(col + 1) * 3 + 1], b01 = src0[(col + 1) * 3 + 0];
                const int r10 = src1[ col      * 3 + 2], g10 = src1[ col      * 3 + 1], b10 = src1[ col      * 3 + 0];
                const int r11 = src1[(col + 1) * 3 + 2], g11 = src1[(col + 1) * 3 + 1], b11 = src1[(col + 1) * 3 + 0];

                const int r_avg = (r00 + r01 + r10 + r11 + 2) >> 2;
                const int g_avg = (g00 + g01 + g10 + g11 + 2) >> 2;
                const int b_avg = (b00 + b01 + b10 + b11 + 2) >> 2;

                u_dst[col / 2] = static_cast<uint8_t>(((-38 * r_avg - 74 * g_avg + 112 * b_avg + 128) >> 8) + 128);
                v_dst[col / 2] = static_cast<uint8_t>(((112 * r_avg - 94 * g_avg -  18 * b_avg + 128) >> 8) + 128);
            }
        }
    }
}

} // namespace kist
