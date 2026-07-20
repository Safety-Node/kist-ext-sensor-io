#pragma once

#include "common/data_buffer.hpp"
#include "realsense/rvl_depth_frame.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace unitree_go::msg::dds_ {
class VoxelMapCompressed_;
}

namespace kist {

// Rx half for compressed depth: subscribes to the DDS byte carrier,
// deserializes back to an RvlDepthFrame, and publishes it to a buffer.
// Deserialize is cheap, so it rides the DDS callback (like the UWB
// receiver); RVL *decode* to a raw DepthFrame is left to the consumer /
// a decode thread. Watchdog clears the buffer after 1s of silence.
class DepthReceiver {
public:
    static DepthReceiver& instance();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kCameraDepthTopic);
    void stop();

    // Optional new-frame hook, run on the DDS receive thread. Keep cheap.
    using OnFrameFn = std::function<void(const RvlDepthFrame&)>;
    void set_on_frame(OnFrameFn fn);

    // Latest compressed frame (read from any thread).
    DataBuffer<RvlDepthFrame> depth_buf;

    // ── internal: DDS callback ──────────────────────────────────
    void on_depth_update(const void* message);

private:
    DepthReceiver() = default;

    void watchdog_loop();

    using Sub = unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::VoxelMapCompressed_>;
    std::unique_ptr<Sub> sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};

    OnFrameFn on_frame_;
};

} // namespace kist
