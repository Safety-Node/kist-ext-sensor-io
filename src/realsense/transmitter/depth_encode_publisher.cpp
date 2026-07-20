#include "realsense/transmitter/depth_encode_publisher.hpp"

#include <pthread.h>

#include <chrono>

namespace kist {

bool DepthEncodePublisher::start(int domain_id, const std::string& network_interface,
                                 DataBuffer<DepthFrame>& source, const std::string& topic) {
    if (running_) return true;
    if (!publisher_.start(domain_id, network_interface, topic))
        return false;
    source_ = &source;
    running_ = true;
    thread_ = std::thread(&DepthEncodePublisher::run, this);
    return true;
}

void DepthEncodePublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void DepthEncodePublisher::run() {
    pthread_setname_np(pthread_self(), "rs-depth-enc");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            publisher_.publish(encoder_.encode(*frame));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
