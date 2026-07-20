#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/h264_color_frame.hpp"
#include "realsense/transmitter/color_publisher.hpp"
#include "realsense/transmitter/h264_color_encoder.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace kist {

// Tx thread (T2): polls a raw-color buffer, H.264-encodes new frames,
// and publishes them. Holds the encoder Module and the DDS publisher;
// owns the worker loop. `source` must outlive this object.
class ColorEncodePublisher {
public:
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<ColorFrame>& source, const H264EncoderConfig& enc_cfg,
               const std::string& topic = kCameraColorTopic);
    void stop();

private:
    void run();

    DataBuffer<ColorFrame>*           source_ = nullptr;
    std::unique_ptr<H264ColorEncoder> encoder_;
    ColorPublisher                    publisher_;
    std::thread                       thread_;
    std::atomic<bool>                 running_{false};
};

} // namespace kist
