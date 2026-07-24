#include "realsense/transmitter/color_publisher.hpp"

#include "kist_camera_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <pthread.h>

#include <chrono>
#include <iostream>

namespace kist {

ColorPublisher::ColorPublisher() = default;
ColorPublisher::~ColorPublisher() { stop(); }

bool ColorPublisher::start_channel(int domain_id, const std::string& network_interface,
                                   const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[ColorPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    return true;
}

bool ColorPublisher::start(int domain_id, const std::string& network_interface,
                           DataBuffer<ColorFrame>& source,
                           const H264EncoderConfig& enc_cfg,
                           const std::string& topic) {
    if (running_) return true;
    if (!start_channel(domain_id, network_interface, topic))
        return false;
    source_  = &source;
    encoder_ = std::make_unique<H264Encoder>(enc_cfg);
    running_ = true;
    thread_  = std::thread(&ColorPublisher::run, this);
    return true;
}

void ColorPublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    encoder_.reset();
    pub_.reset();
}

void ColorPublisher::publish(const H264ColorFrame& frame) {
    kist_msgs::CompressedColorFrame msg;
    msg.width(uint32_t(frame.width));
    msg.height(uint32_t(frame.height));
    msg.seq(frame.sequence);
    msg.stamp_ns(frame.stamp_ns);
    msg.is_keyframe(frame.is_keyframe);
    msg.frame_id(frame.frame_id);
    msg.data(frame.data);
    pub_->Write(msg);
}

void ColorPublisher::run() {
    pthread_setname_np(pthread_self(), "rs-color-enc");
    int64_t last_stamp = -1;
    while (running_) {
        auto frame = source_->GetData();
        if (frame && frame->stamp_ns != last_stamp) {
            last_stamp = frame->stamp_ns;
            if (auto enc = encoder_->encode(*frame))
                publish(*enc);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

} // namespace kist
