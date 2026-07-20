#pragma once

#include "common/data_buffer.hpp"
#include "uwb/uwb_position.hpp"
#include "uwb/uwb_sample.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

// Publisher thread of the Tx side: polls a sample buffer (10ms) and
// publishes NEW samples as PoseStamped over DDS, plus a once-per-second
// status line. Orientation is identity — UWB has no heading source.
//
// Publish-on-new only: when the source goes quiet (tag lost its fix,
// serial died), the topic goes honestly silent and downstream buffers
// stale (empty-buffer principle).
class UwbPublisher {
public:
    UwbPublisher();
    ~UwbPublisher();

    // Full role: pump `source` onto the wire until stop().
    // `source` must outlive this object (the transmitter owns both).
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<UwbSample>& source, const std::string& frame_id,
               const std::string& topic = kUwbPoseTopic);
    void stop();

    // ── manual injection (tests / synthetic sources) ────────────
    // DDS side only, no source polling; then publish() by hand.
    bool start_channel(int domain_id, const std::string& network_interface,
                       const std::string& topic = kUwbPoseTopic);
    void publish(const UwbSample& sample, const std::string& frame_id);

private:
    void publish_loop(DataBuffer<UwbSample>* source, std::string frame_id);

    using PosePub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<PosePub> pub_;

    std::thread       publish_thread_;
    std::atomic<bool> stop_publish_{false};
};

} // namespace kist
