#include "uwb/receiver/uwb_subscriber.hpp"

#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

UwbSubscriber::UwbSubscriber() = default;
UwbSubscriber::~UwbSubscriber() { stop(); }

bool UwbSubscriber::start(int domain_id, const std::string& network_interface,
                      const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        pose_sub_.reset(new PoseSub(topic));
        pose_sub_->InitChannel(
            [this](const void* msg) { on_pose_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UwbSubscriber] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UwbSubscriber::watchdog_loop, this);
    std::cout << "[UwbSubscriber] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void UwbSubscriber::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    pose_sub_.reset();
}

void UwbSubscriber::set_on_position(OnPositionFn fn) {
    on_position_ = std::move(fn);
}

void UwbSubscriber::on_pose_update(const void* message) {
    const auto& msg = *static_cast<const geometry_msgs::msg::dds_::PoseStamped_*>(message);

    UwbPosition fix;
    fix.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                   msg.header().stamp().nanosec();
    fix.x = float(msg.pose().position().x());
    fix.y = float(msg.pose().position().y());
    fix.z = float(msg.pose().position().z());

    uwb_buf.SetData(UwbPosition(fix));
    if (on_position_)
        on_position_(fix);
}

// Fixes stream at ~10Hz while the tag has anchor coverage; 1s of
// silence (10 frames) clears the buffer so consumers see "no fix" and
// degrade (e.g. the EKF falls back to odom dead-reckoning). Polled at
// 10ms, same cadence as the other readers' watchdogs.
void UwbSubscriber::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 1000.0;  // 10 frames at 10Hz

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto fix = uwb_buf.GetDataWithTime();
        if (fix.HasData() && fix.GetAgeMs() > stale_ms) {
            std::cerr << "[UwbSubscriber] UWB fixes stale — cleared\n";
            uwb_buf.Clear();
        }
    }
}

} // namespace kist
