#include "realsense/receiver/depth_subscriber.hpp"

#include "kist_camera_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

DepthSubscriber::DepthSubscriber() = default;
DepthSubscriber::~DepthSubscriber() { stop(); }

bool DepthSubscriber::start(int domain_id, const std::string& network_interface,
                          const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        sub_.reset(new Sub(topic));
        // Reader queue depth (KEEP_LAST N). Deep enough (~1s at 30fps) to absorb
        // arrival bursts without dropping at the reader; the downstream buffer is
        // latest-wins + stamp-deduped, so a deeper queue adds no consumer latency.
        sub_->InitChannel(
            [this](const void* msg) { on_depth_update(msg); }, 30);
    } catch (const std::exception& e) {
        std::cerr << "[DepthSubscriber] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&DepthSubscriber::watchdog_loop, this);
    std::cout << "[DepthSubscriber] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void DepthSubscriber::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    sub_.reset();
}

void DepthSubscriber::set_on_frame(OnFrameFn fn) {
    on_frame_ = std::move(fn);
}

void DepthSubscriber::on_depth_update(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::CompressedDepthFrame*>(message);

    RvlDepthFrame frame;
    frame.width       = int(msg.width());
    frame.height      = int(msg.height());
    frame.sequence    = msg.seq();
    frame.stamp_ns    = msg.stamp_ns();
    frame.depth_scale = msg.depth_scale();
    frame.fx          = msg.fx();
    frame.fy          = msg.fy();
    frame.cx          = msg.cx();
    frame.cy          = msg.cy();
    frame.frame_id    = msg.frame_id();
    frame.data        = msg.data();

    depth_buf.SetData(RvlDepthFrame(frame));
    if (on_frame_)
        on_frame_(frame);
}

// Camera streams at ~30Hz; 1s of silence clears the buffer so consumers
// see "no frame". Polled at 10ms, same cadence as the other watchdogs.
void DepthSubscriber::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 1000.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto frame = depth_buf.GetDataWithTime();
        if (frame.HasData() && frame.GetAgeMs() > stale_ms) {
            std::cerr << "[DepthSubscriber] depth frames stale — cleared\n";
            depth_buf.Clear();
        }
    }
}

} // namespace kist
