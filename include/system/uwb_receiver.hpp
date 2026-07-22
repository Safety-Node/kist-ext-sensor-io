#pragma once

#include "common/data_buffer.hpp"
#include "uwb/uwb_position.hpp"
#include "uwb/receiver/uwb_subscriber.hpp"

#include <utility>

namespace kist {

// UWB Rx assembly (own-no-thread) — the mirror of UwbTransmitter in system/.
// UWB has no decode stage, so this owns the single Rx unit (UwbSubscriber)
// and just manages its lifecycle; it exists for a uniform embedding API with
// RealsenseReceiver. Consumers link this, start it, and read fixes in-process
// via fix(). Empty buffer = no live fix (1s watchdog on the subscriber).
class UwbReceiver {
public:
    bool start(int domain_id, const std::string& network_interface);
    void stop();

    // Latest UWB fix (read from any thread).
    DataBuffer<UwbPosition>& fix() { return sub_.uwb_buf; }

    // Optional new-fix hook (runs on the DDS receive thread — keep cheap).
    // Set before start().
    void set_on_position(UwbSubscriber::OnPositionFn fn) {
        sub_.set_on_position(std::move(fn));
    }

private:
    UwbSubscriber sub_;
    bool          running_ = false;
};

} // namespace kist
