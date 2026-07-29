#include "realsense/receiver/color_decoder.hpp"

#include <pthread.h>

#include <chrono>
#include <stdexcept>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace kist {

// ── H264ColorDecoder (libav Module) ──────────────────────────────────

struct H264ColorDecoder::Impl {
    AVCodecContext* ctx     = nullptr;
    AVPacket*       pkt     = nullptr;
    AVFrame*        frame   = nullptr;
    SwsContext*     sws     = nullptr;
    int             sws_w   = 0;
    int             sws_h   = 0;
};

H264ColorDecoder::H264ColorDecoder() : impl_(std::make_unique<Impl>()) {
    av_log_set_level(AV_LOG_FATAL);

    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) throw std::runtime_error("H264ColorDecoder: H264 decoder not found");

    impl_->ctx = avcodec_alloc_context3(codec);
    if (!impl_->ctx) throw std::runtime_error("H264ColorDecoder: alloc_context3 failed");

    if (avcodec_open2(impl_->ctx, codec, nullptr) < 0) {
        avcodec_free_context(&impl_->ctx);
        throw std::runtime_error("H264ColorDecoder: avcodec_open2 failed");
    }

    impl_->pkt   = av_packet_alloc();
    impl_->frame = av_frame_alloc();
    if (!impl_->pkt || !impl_->frame) {
        av_frame_free(&impl_->frame);
        av_packet_free(&impl_->pkt);
        avcodec_free_context(&impl_->ctx);
        throw std::runtime_error("H264ColorDecoder: packet/frame alloc failed");
    }
}

H264ColorDecoder::~H264ColorDecoder() {
    if (impl_->sws)   sws_freeContext(impl_->sws);
    if (impl_->frame) av_frame_free(&impl_->frame);
    if (impl_->pkt)   av_packet_free(&impl_->pkt);
    if (impl_->ctx)   avcodec_free_context(&impl_->ctx);
}

std::optional<ColorFrame> H264ColorDecoder::decode(const H264ColorFrame& frame) {
    av_packet_unref(impl_->pkt);
    impl_->pkt->data = const_cast<uint8_t*>(frame.data.data());
    impl_->pkt->size = static_cast<int>(frame.data.size());

    if (avcodec_send_packet(impl_->ctx, impl_->pkt) < 0)
        return std::nullopt;

    const int ret = avcodec_receive_frame(impl_->ctx, impl_->frame);
    if (ret < 0)  // EAGAIN (needs more input) / EOF / error
        return std::nullopt;

    const int w = impl_->frame->width;
    const int h = impl_->frame->height;
    const auto src_fmt = static_cast<AVPixelFormat>(impl_->frame->format);

    if (!impl_->sws || impl_->sws_w != w || impl_->sws_h != h) {
        if (impl_->sws) sws_freeContext(impl_->sws);
        impl_->sws = sws_getContext(w, h, src_fmt, w, h, AV_PIX_FMT_BGR24,
                                    SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!impl_->sws) throw std::runtime_error("H264ColorDecoder: sws_getContext failed");
        impl_->sws_w = w;
        impl_->sws_h = h;
    }

    ColorFrame result;
    result.width        = w;
    result.height       = h;
    result.stride_bytes = w * 3;
    result.stamp_ns     = frame.stamp_ns;
    result.frame_id     = frame.frame_id;
    result.data.resize(static_cast<size_t>(h) * w * 3);

    uint8_t* dst[1]        = { result.data.data() };
    int      dst_stride[1] = { result.stride_bytes };
    sws_scale(impl_->sws, impl_->frame->data, impl_->frame->linesize, 0, h, dst, dst_stride);

    return result;
}

// ── ColorDecodeThread ────────────────────────────────────────────────

bool ColorDecodeThread::start(DataBuffer<H264ColorFrame>& source) {
    if (running_) return true;
    source_ = &source;
    decoder_ = std::make_unique<H264ColorDecoder>();
    running_ = true;
    thread_ = std::thread(&ColorDecodeThread::run, this);
    return true;
}

void ColorDecodeThread::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    decoder_.reset();
}

void ColorDecodeThread::run() {
    pthread_setname_np(pthread_self(), "rs-color-dec");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            if (auto dec = decoder_->decode(*frame)) {
                out.SetData(std::move(*dec));
                decoded_.fetch_add(1, std::memory_order_relaxed);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
