#include "realsense/receiver/depth_decode_thread.hpp"

#include <pthread.h>

#include <chrono>

namespace kist {

bool DepthDecodeThread::start(DataBuffer<RvlDepthFrame>& source) {
    if (running_) return true;
    source_ = &source;
    running_ = true;
    thread_ = std::thread(&DepthDecodeThread::run, this);
    return true;
}

void DepthDecodeThread::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void DepthDecodeThread::run() {
    pthread_setname_np(pthread_self(), "rs-depth-dec");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            out.SetData(decoder_.decode(*frame));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
