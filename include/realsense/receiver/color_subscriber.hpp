#pragma once

#include "common/data_buffer.hpp"
#include "realsense/h264_color_frame.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace kist_msgs {
class CompressedColorFrame;
}

namespace kist {

// Rx half for compressed color — the color twin of DepthSubscriber:
// subscribes to the idlc-generated kist_msgs::CompressedColorFrame,
// maps it to an H264ColorFrame, publishes it to a buffer. The mapping is
// cheap, so it rides the DDS callback; H.264 *decode* is left to the
// consumer / a decode thread. Watchdog clears the buffer after 1s.
class ColorSubscriber {
public:
    ColorSubscriber();
    ~ColorSubscriber();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kCameraColorTopic);
    void stop();

    using OnFrameFn = std::function<void(const H264ColorFrame&)>;
    void set_on_frame(OnFrameFn fn);

    DataBuffer<H264ColorFrame> color_buf;

    void on_color_update(const void* message);

private:
    void watchdog_loop();

    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::CompressedColorFrame>;
    std::unique_ptr<Sub> sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};

    OnFrameFn on_frame_;
};

} // namespace kist
