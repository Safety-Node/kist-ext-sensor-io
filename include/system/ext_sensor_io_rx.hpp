#pragma once

#include "uwb/uwb_receiver.hpp"

#include <string>

namespace kist {

// Rx-side assembly — the consumer-machine counterpart of ExtSensorIoTx.
// One start() brings up the Receiver of every sensor enabled in the
// config; data access stays per sensor (each Receiver's buffer/hook),
// so adding sensors never changes the embedding code:
//
//   auto& rx = kist::ExtSensorIoRx::instance();
//   rx.start("config/config.yaml");
//   auto fix = rx.uwb().uwb_buf.GetDataWithTime();
//   ...
//   rx.stop();
class ExtSensorIoRx {
public:
    static ExtSensorIoRx& instance();

    // Starts every sensor configured in the file (a sensor is skipped
    // when its section is absent or has `enabled: false`). Returns
    // false when nothing could start.
    bool start(const std::string& config_path = "config/config.yaml");
    void stop();

    // ── per-sensor access (buffers + hooks live on the Receivers) ──
    UwbReceiver& uwb() { return UwbReceiver::instance(); }

private:
    ExtSensorIoRx() = default;

    bool uwb_running_ = false;
};

} // namespace kist
