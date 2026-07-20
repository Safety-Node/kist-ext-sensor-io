#pragma once

#include "common/data_buffer.hpp"
#include "uwb/uwb_position.hpp"  // UwbPosition + kUwbPoseTopic

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

// Rx half: UWB fixes from the DDS wire into a DataBuffer — the same
// subscribe pattern as kist-navigation-planner's unitree readers.
//
// UWB silence is a legitimate state (tag out of anchor range, or the
// publisher dropped invalid fixes), so the watchdog clears the buffer
// after 1s and consumers key off "has data" (empty-buffer principle).
class UwbReceiver {
public:
    static UwbReceiver& instance();

    // One-liner form: reads domain_id / network_interface from the
    // config file's `unitree:` section (gearsonic-style facade).
    bool start(const std::string& config_path = "config/config.yaml");

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kUwbPoseTopic);
    void stop();

    // Optional new-fix hook, run inside the DDS receive callback right
    // after uwb_buf is updated. This is how an integrating wrapper wires
    // fixes into another module's passive buffer without polling. Must
    // stay cheap — it blocks the receive thread. Set before start();
    // not guarded against mutation while fixes are flowing.
    using OnPositionFn = std::function<void(const UwbPosition&)>;
    void set_on_position(OnPositionFn fn);

    // ── data buffer (read from any thread) ─────────────────────
    DataBuffer<UwbPosition> uwb_buf;

    // ── internal: DDS callback ──────────────────────────────────
    void on_pose_update(const void* message);

private:
    UwbReceiver() = default;

    void watchdog_loop();

    using PoseSub = unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<PoseSub> pose_sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};

    OnPositionFn on_position_;
};

} // namespace kist
