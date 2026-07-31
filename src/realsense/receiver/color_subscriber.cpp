#include "realsense/receiver/color_subscriber.hpp"

#include "kist_camera_frames.hpp"  // idlc-generated

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

ColorSubscriber::ColorSubscriber() = default;
ColorSubscriber::~ColorSubscriber() { stop(); }

bool ColorSubscriber::start(int domain_id, const std::string& network_interface,
                          const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_.reset(new Sub(topic));
        // Reader queue depth (KEEP_LAST N). Deep enough (~1s at 30fps) to absorb
        // arrival bursts without dropping at the reader; the downstream buffer is
        // latest-wins + stamp-deduped, so a deeper queue adds no consumer latency.
        sub_->InitChannel(
            [this](const void* msg) { on_color_update(msg); }, 30);
    } catch (const std::exception& e) {
        std::cerr << "[ColorSubscriber] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&ColorSubscriber::watchdog_loop, this);
    std::cout << "[ColorSubscriber] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void ColorSubscriber::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    sub_.reset();
}

void ColorSubscriber::set_on_frame(OnFrameFn fn) {
    on_frame_ = std::move(fn);
}

void ColorSubscriber::on_color_update(const void* message) {
    const auto& msg = *static_cast<const kist_msgs::CompressedColorFrame*>(message);

    H264ColorFrame frame;
    frame.width       = int(msg.width());
    frame.height      = int(msg.height());
    frame.sequence    = msg.seq();
    frame.stamp_ns    = msg.stamp_ns();
    frame.is_keyframe = msg.is_keyframe();
    frame.frame_id    = msg.frame_id();
    frame.data        = msg.data();

    color_buf.SetData(H264ColorFrame(frame));
    if (on_frame_)
        on_frame_(frame);
}

// Camera streams at ~30Hz; 1s of silence clears the buffer so consumers
// see "no frame". Polled at 10ms, same cadence as the other watchdogs.
void ColorSubscriber::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 1000.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto frame = color_buf.GetDataWithTime();
        if (frame.HasData() && frame.GetAgeMs() > stale_ms) {
            std::cerr << "[ColorSubscriber] color frames stale — cleared\n";
            color_buf.Clear();
        }
    }
}

} // namespace kist
