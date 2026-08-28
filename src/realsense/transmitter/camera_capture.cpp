#include "realsense/transmitter/camera_capture.hpp"

#include <librealsense2/rs.hpp>

#include <cstring>
#include <iostream>
#include <optional>

namespace kist {

CameraCapture::CameraCapture() = default;
CameraCapture::~CameraCapture() { stop(); }

bool CameraCapture::start(const CameraCaptureConfig& config) {
    if (running_) return true;
    config_ = config;

    try {
        rs2::config cfg;
        if (!config_.serial.empty())
            cfg.enable_device(config_.serial);   // pick this exact camera
        cfg.enable_stream(RS2_STREAM_DEPTH,
                          config_.depth_width, config_.depth_height,
                          RS2_FORMAT_Z16, config_.depth_fps);
        if (config_.color_enabled)
            cfg.enable_stream(RS2_STREAM_COLOR,
                              config_.color_width, config_.color_height,
                              RS2_FORMAT_BGR8, config_.color_fps);

        pipe_.reset(new rs2::pipeline());
        rs2::pipeline_profile profile = pipe_->start(cfg);
        try {
            depth_scale_ = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
        } catch (...) {}
    } catch (const std::exception& e) {
        std::cerr << "[CameraCapture] RealSense start failed: " << e.what()
                  << "\n  Check the camera USB connection and requested modes.\n";
        pipe_.reset();
        return false;
    }

    running_ = true;
    worker_  = std::thread(&CameraCapture::run, this);
    std::cout << "[CameraCapture] started depth "
              << config_.depth_width << "x" << config_.depth_height
              << "@" << config_.depth_fps << "  depth_scale=" << depth_scale_ << "\n";
    return true;
}

void CameraCapture::stop() {
    running_ = false;
    if (worker_.joinable())
        worker_.join();
    if (pipe_) {
        try { pipe_->stop(); } catch (...) {}
        pipe_.reset();
    }
}

void CameraCapture::run() {
    std::optional<rs2::align> align;
    if (config_.align_to_color)
        align.emplace(RS2_STREAM_COLOR);

    std::optional<rs2::spatial_filter> spatial;
    if (config_.spatial_filter.enabled) {
        const auto& sf = config_.spatial_filter;
        spatial.emplace();
        spatial->set_option(RS2_OPTION_FILTER_MAGNITUDE,    float(sf.magnitude));
        spatial->set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, sf.smooth_alpha);
        spatial->set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, sf.smooth_delta);
    }

    while (running_) {
        try {
            rs2::frameset frames;
            if (!pipe_->try_wait_for_frames(&frames, 100))
                continue;

            const uint64_t seq = sequence_.fetch_add(1);
            rs2::frameset fs = align ? align->process(frames) : frames;

            if (rs2::depth_frame df = fs.get_depth_frame()) {
                if (spatial)
                    df = spatial->process(df);
                const int w = df.get_width(), h = df.get_height();
                const int stride = df.get_stride_in_bytes(), size = df.get_data_size();
                if (w > 0 && h > 0 && stride > 0 && size > 0) {
                    DepthFrame frame;
                    frame.width = w; frame.height = h; frame.stride_bytes = stride;
                    frame.sequence = seq;
                    frame.stamp_ns = static_cast<int64_t>(df.get_timestamp() * 1'000'000.0);
                    frame.depth_scale = depth_scale_;
                    // Intrinsics of the frame as published: after align_to_color
                    // this profile carries the color stream's intrinsics, which is
                    // exactly what deprojects the aligned depth into 3D.
                    try {
                        const auto in = df.get_profile()
                                          .as<rs2::video_stream_profile>()
                                          .get_intrinsics();
                        frame.fx = in.fx; frame.fy = in.fy;
                        frame.cx = in.ppx; frame.cy = in.ppy;
                    } catch (...) {}
                    frame.frame_id = config_.depth_frame_id;
                    frame.data.resize(size);
                    std::memcpy(frame.data.data(), df.get_data(), size);
                    depth_buf.SetData(std::move(frame));
                }
            }

            if (config_.color_enabled) {
                if (rs2::video_frame vf = fs.get_color_frame()) {
                    const int w = vf.get_width(), h = vf.get_height();
                    const int stride = vf.get_stride_in_bytes(), size = vf.get_data_size();
                    if (w > 0 && h > 0 && stride > 0 && size > 0) {
                        ColorFrame frame;
                        frame.width = w; frame.height = h; frame.stride_bytes = stride;
                        frame.sequence = seq;
                        frame.stamp_ns = static_cast<int64_t>(vf.get_timestamp() * 1'000'000.0);
                        frame.frame_id = config_.color_frame_id;
                        frame.data.resize(size);
                        std::memcpy(frame.data.data(), vf.get_data(), size);
                        color_buf.SetData(std::move(frame));
                    }
                }
            }
        } catch (const std::exception& e) {
            // A device hiccup (USB drop / disconnect) makes librealsense throw
            // from try_wait_for_frames; don't take the whole process down with
            // it. Log and stop capturing cleanly — downstream buffers then go
            // stale (the honest "no live frame" state) instead of a core dump.
            std::cerr << "[CameraCapture] capture stopped on RealSense error: "
                      << e.what() << "\n";
            running_ = false;
        }
    }
}

} // namespace kist
