#pragma once

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/color_encode_publisher.hpp"
#include "realsense/transmitter/depth_encode_publisher.hpp"
#include "realsense/transmitter/h264_encoder.hpp"

#include <string>

namespace kist {

// Tx-side assembly (owns no thread of its own) — wires the three camera
// Tx threads:
//   [CameraCapture T1]  -> color_buf / depth_buf
//   [ColorEncodePublisher T2]  color_buf -> H.264 -> rt/kist/camera/color
//   [DepthEncodePublisher T3]  depth_buf -> RVL   -> rt/kist/camera/depth
// Runs on the machine with the RealSense (the robot side).
class CameraTransmitter {
public:
    bool start(int domain_id, const std::string& network_interface,
               const CameraCaptureConfig& capture_cfg,
               const H264EncoderConfig& color_cfg);
    void stop();

private:
    CameraCapture        capture_;
    ColorEncodePublisher color_;
    DepthEncodePublisher depth_;
    bool                 running_ = false;
};

} // namespace kist
