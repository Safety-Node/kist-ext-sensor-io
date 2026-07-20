#include "realsense/receiver/color_decode_thread.hpp"

#include <pthread.h>

#include <chrono>

namespace kist {

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
            if (auto dec = decoder_->decode(*frame))
                out.SetData(std::move(*dec));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
