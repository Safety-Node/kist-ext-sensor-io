#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"
#include "realsense/transmitter/h264_encoder.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace kist_msgs {
class CompressedColorFrame;
}

namespace kist {

// Tx thread (T2): polls a raw-color buffer, H.264-encodes new frames (via
// the H264Encoder Module — see h264_encoder.hpp), and publishes them as
// typed DDS messages (kist_msgs::CompressedColorFrame). Encode and publish
// run together in this one thread. `source` must outlive this object.
class ColorPublisher {
public:
    ColorPublisher();
    ~ColorPublisher();

    // Full pipeline: opens the DDS channel, then runs the encode+publish
    // worker off `source`.
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<ColorFrame>& source, const H264EncoderConfig& enc_cfg,
               const std::string& topic = kCameraColorTopic);
    void stop();

    // DDS channel only (no worker) + one-shot publish — for tests / manual
    // sources that already hold an encoded frame.
    bool start_channel(int domain_id, const std::string& network_interface,
                       const std::string& topic = kCameraColorTopic);
    void publish(const H264ColorFrame& frame);

private:
    void run();

    using Pub = unitree::robot::ChannelPublisher<kist_msgs::CompressedColorFrame>;

    DataBuffer<ColorFrame>*      source_ = nullptr;
    std::unique_ptr<H264Encoder> encoder_;
    std::unique_ptr<Pub>         pub_;
    std::thread                  thread_;
    std::atomic<bool>            running_{false};
};

} // namespace kist
