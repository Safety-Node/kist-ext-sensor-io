#pragma once

#include "realsense/transmitter/camera_capture.hpp"
#include "realsense/transmitter/color_publisher.hpp"
#include "realsense/transmitter/depth_publisher.hpp"
#include "realsense/transmitter/h264_encoder.hpp"

#include <string>

namespace kist {

// RealSense camera Tx assembly (owns no thread of its own) — wires the
// three Tx threads and manages their lifecycle:
//   [CameraCapture]   -> color_buf / depth_buf
//   [ColorPublisher]  color_buf -> H.264 -> rt/kist/camera/<name>/color/h264
//   [DepthPublisher]  depth_buf -> RVL   -> rt/kist/camera/<name>/depth/rvl
// One instance per physical camera: `name` namespaces its DDS topics and
// capture_cfg.serial picks the device. Embed it in a device-side app (pass
// configs directly), or run it via the test_realsense_transmitter executable
// (which spawns one per configured camera). Config parsing stays out of here.
class RealsenseTransmitter {
public:
    bool start(int domain_id, const std::string& network_interface,
               const std::string& name,
               const CameraCaptureConfig& capture_cfg,
               const H264EncoderConfig& color_cfg);
    void stop();

    // Frames written to DDS so far (monotonic). Poll the delta over a window
    // for the publish rate per stream — the produce-side truth for the
    // Tx→Rx→consumer fps comparison.
    uint64_t color_published() const { return color_.published(); }
    uint64_t depth_published() const { return depth_.published(); }

private:
    CameraCapture  capture_;
    ColorPublisher color_;
    DepthPublisher depth_;
    bool           running_ = false;
};

} // namespace kist
