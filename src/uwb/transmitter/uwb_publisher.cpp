#include "uwb/transmitter/uwb_publisher.hpp"

#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <pthread.h>

#include <chrono>
#include <iostream>

namespace kist {

UwbPublisher::UwbPublisher() = default;
UwbPublisher::~UwbPublisher() { stop(); }

bool UwbPublisher::start_channel(int domain_id, const std::string& network_interface,
                                 const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        pub_.reset(new PosePub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[UwbPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    return true;
}

bool UwbPublisher::start(int domain_id, const std::string& network_interface,
                         DataBuffer<UwbSample>& source, const std::string& frame_id,
                         const std::string& topic) {
    if (!start_channel(domain_id, network_interface, topic))
        return false;

    stop_publish_ = false;
    publish_thread_ = std::thread(&UwbPublisher::publish_loop, this, &source, frame_id);
    return true;
}

void UwbPublisher::stop() {
    stop_publish_ = true;
    if (publish_thread_.joinable())
        publish_thread_.join();
    pub_.reset();
}

void UwbPublisher::publish(const UwbSample& sample, const std::string& frame_id) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    geometry_msgs::msg::dds_::PoseStamped_ msg;
    msg.header().stamp().sec(int32_t(ns / 1000000000LL));
    msg.header().stamp().nanosec(uint32_t(ns % 1000000000LL));
    msg.header().frame_id(frame_id);
    msg.pose().position().x(sample.x);
    msg.pose().position().y(sample.y);
    msg.pose().position().z(sample.z);
    msg.pose().orientation().x(0.0);
    msg.pose().orientation().y(0.0);
    msg.pose().orientation().z(0.0);
    msg.pose().orientation().w(1.0);  // no heading source

    pub_->Write(msg);
}

void UwbPublisher::publish_loop(DataBuffer<UwbSample>* source, std::string frame_id) {
    pthread_setname_np(pthread_self(), "uwb-publisher");  // visible in htop/gdb
    using clock = std::chrono::steady_clock;

    auto last_sample_time = clock::time_point{};

    // Publish-on-new only; the library stays silent (device-side status is
    // the runner's job — see test_uwb_transmitter).
    while (!stop_publish_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto sample = source->GetDataWithTime();
        if (sample.HasData() && sample.timestamp != last_sample_time) {
            last_sample_time = sample.timestamp;
            publish(*sample.data, frame_id);
        }
    }
}

} // namespace kist
