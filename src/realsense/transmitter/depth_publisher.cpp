#include "realsense/transmitter/depth_publisher.hpp"

#include "kist_camera_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <pthread.h>

#include <chrono>
#include <iostream>

namespace kist {

DepthPublisher::DepthPublisher() = default;
DepthPublisher::~DepthPublisher() { stop(); }

bool DepthPublisher::start_channel(int domain_id, const std::string& network_interface,
                                   const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[DepthPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    return true;
}

bool DepthPublisher::start(int domain_id, const std::string& network_interface,
                           DataBuffer<DepthFrame>& source, const std::string& topic) {
    if (running_) return true;
    if (!start_channel(domain_id, network_interface, topic))
        return false;
    source_  = &source;
    running_ = true;
    thread_  = std::thread(&DepthPublisher::run, this);
    return true;
}

void DepthPublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    pub_.reset();
}

void DepthPublisher::publish(const RvlDepthFrame& frame) {
    kist_msgs::CompressedDepthFrame msg;
    msg.width(uint32_t(frame.width));
    msg.height(uint32_t(frame.height));
    msg.seq(frame.sequence);
    msg.stamp_ns(frame.stamp_ns);
    msg.depth_scale(frame.depth_scale);
    msg.fx(frame.fx); msg.fy(frame.fy);
    msg.cx(frame.cx); msg.cy(frame.cy);
    msg.frame_id(frame.frame_id);
    msg.data(frame.data);
    pub_->Write(msg);
    published_.fetch_add(1, std::memory_order_relaxed);
}

void DepthPublisher::run() {
    pthread_setname_np(pthread_self(), "rs-depth-enc");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            publish(encoder_.encode(*frame));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
