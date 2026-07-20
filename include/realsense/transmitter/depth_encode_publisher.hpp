#pragma once

#include "common/data_buffer.hpp"
#include "realsense/depth_frame.hpp"
#include "realsense/transmitter/depth_publisher.hpp"
#include "realsense/transmitter/rvl_depth_encoder.hpp"

#include <atomic>
#include <string>
#include <thread>

namespace kist {

// Tx thread (T3): polls a raw-depth buffer, RVL-encodes new frames, and
// publishes them. Holds the encoder Module and the DDS publisher; owns
// the worker loop. `source` must outlive this object.
class DepthEncodePublisher {
public:
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<DepthFrame>& source,
               const std::string& topic = kCameraDepthTopic);
    void stop();

private:
    void run();

    DataBuffer<DepthFrame>* source_ = nullptr;
    RvlDepthEncoder         encoder_;
    DepthPublisher          publisher_;
    std::thread             thread_;
    std::atomic<bool>       running_{false};
};

} // namespace kist
