#include "realsense/transmitter/color_encode_publisher.hpp"

#include <pthread.h>

#include <chrono>

namespace kist {

bool ColorEncodePublisher::start(int domain_id, const std::string& network_interface,
                                 DataBuffer<ColorFrame>& source,
                                 const H264EncoderConfig& enc_cfg,
                                 const std::string& topic) {
    if (running_) return true;
    if (!publisher_.start(domain_id, network_interface, topic))
        return false;
    source_ = &source;
    encoder_ = std::make_unique<H264ColorEncoder>(enc_cfg);
    running_ = true;
    thread_ = std::thread(&ColorEncodePublisher::run, this);
    return true;
}

void ColorEncodePublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    encoder_.reset();
}

void ColorEncodePublisher::run() {
    pthread_setname_np(pthread_self(), "rs-color-enc");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            if (auto enc = encoder_->encode(*frame))
                publisher_.publish(*enc);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
