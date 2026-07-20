#include "realsense/receiver/color_receiver.hpp"

#include "realsense/h264_color_wire.hpp"

#include <unitree/idl/go2/VoxelMapCompressed_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

ColorReceiver& ColorReceiver::instance() {
    static ColorReceiver inst;
    return inst;
}

bool ColorReceiver::start(int domain_id, const std::string& network_interface,
                          const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        sub_.reset(new Sub(topic));
        sub_->InitChannel(
            [this](const void* msg) { on_color_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[ColorReceiver] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&ColorReceiver::watchdog_loop, this);
    std::cout << "[ColorReceiver] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void ColorReceiver::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    sub_.reset();
}

void ColorReceiver::set_on_frame(OnFrameFn fn) {
    on_frame_ = std::move(fn);
}

void ColorReceiver::on_color_update(const void* message) {
    const auto& msg = *static_cast<const unitree_go::msg::dds_::VoxelMapCompressed_*>(message);

    H264ColorFrame frame = deserialize_color(msg.data());
    color_buf.SetData(H264ColorFrame(frame));
    if (on_frame_)
        on_frame_(frame);
}

// Camera streams at ~30Hz; 1s of silence clears the buffer so consumers
// see "no frame". Polled at 10ms, same cadence as the other watchdogs.
void ColorReceiver::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 1000.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto frame = color_buf.GetDataWithTime();
        if (frame.HasData() && frame.GetAgeMs() > stale_ms) {
            std::cerr << "[ColorReceiver] color frames stale — cleared\n";
            color_buf.Clear();
        }
    }
}

} // namespace kist
