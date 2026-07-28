#pragma once

#include "common/data_buffer.hpp"
#include "realsense/color_frame.hpp"
#include "realsense/depth_frame.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace rs2 { class pipeline; }

namespace kist {

// Optional RealSense spatial (edge-preserving) filter on depth. Disabled
// unless enabled; params match librealsense's RS2_OPTION_FILTER_*.
struct SpatialFilterConfig {
    bool  enabled      = false;
    int   magnitude    = 2;      // filter passes, 1..5
    float smooth_alpha = 0.5f;   // neighbor weight, 0..1
    float smooth_delta = 20.0f;  // depth-discontinuity tolerance (mm)
};

struct CameraCaptureConfig {
    // RealSense device serial to open. Empty = first available device. Required
    // to disambiguate when several cameras are attached (multi-camera setups).
    std::string serial;

    int  depth_width  = 640;
    int  depth_height = 480;
    int  depth_fps    = 30;

    bool color_enabled = true;
    int  color_width   = 640;
    int  color_height  = 480;
    int  color_fps     = 30;

    bool align_to_color = true;  // reproject depth into the color frame

    SpatialFilterConfig spatial_filter;

    std::string depth_frame_id = "camera_depth";
    std::string color_frame_id = "camera_color";
};

// Camera device reader (Thread) — the RealSense counterpart of
// UwbSerialReader. Owns a worker that pulls framesets from a librealsense
// pipeline and writes raw ColorFrame / DepthFrame to buffers. No codec,
// no DDS here; encode+publish threads consume these buffers.
class CameraCapture {
public:
    CameraCapture();
    ~CameraCapture();

    bool start(const CameraCaptureConfig& config);
    void stop();

    DataBuffer<DepthFrame> depth_buf;
    DataBuffer<ColorFrame> color_buf;

private:
    void run();

    CameraCaptureConfig      config_;
    std::unique_ptr<rs2::pipeline> pipe_;
    float                    depth_scale_ = 0.001f;
    std::atomic<bool>        running_{false};
    std::atomic<uint64_t>    sequence_{0};
    std::thread              worker_;
};

} // namespace kist
