#pragma once

#include "common/data_buffer.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/rvl_depth_frame.hpp"
#include "realsense/transmitter/rvl_encoder.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace kist_msgs {
class CompressedDepthFrame;
}

namespace kist {

// Tx thread (T3): polls a raw-depth buffer, RVL-encodes new frames (via the
// RvlEncoder Module — see rvl_encoder.hpp), and publishes them as typed DDS
// messages (kist_msgs::CompressedDepthFrame). Encode and publish run
// together in this one thread. `source` must outlive this object.
class DepthPublisher {
public:
    DepthPublisher();
    ~DepthPublisher();

    // Full pipeline: opens the DDS channel, then runs the encode+publish
    // worker off `source`.
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<DepthFrame>& source,
               const std::string& topic = kCameraDepthTopic);
    void stop();

    // DDS channel only (no worker) + one-shot publish — for tests / manual
    // sources that already hold an encoded frame.
    bool start_channel(int domain_id, const std::string& network_interface,
                       const std::string& topic = kCameraDepthTopic);
    void publish(const RvlDepthFrame& frame);

private:
    void run();

    using Pub = unitree::robot::ChannelPublisher<kist_msgs::CompressedDepthFrame>;

    DataBuffer<DepthFrame>* source_ = nullptr;
    RvlEncoder              encoder_;
    std::unique_ptr<Pub>    pub_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};
};

} // namespace kist
