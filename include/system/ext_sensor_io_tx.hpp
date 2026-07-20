#pragma once

#include "uwb/uwb_transmitter.hpp"

#include <string>

namespace kist {

// Tx-side assembly — lifecycle only, the mirror of ExtSensorIoRx. One
// process on the device machine (e.g. the G1 onboard PC) starts the
// Transmitter of every sensor enabled in the single config file; the
// actual work (serial, publisher loop, DDS) lives in each sensor's Transmitter.
//
//   auto& tx = kist::ExtSensorIoTx::instance();
//   tx.start("config/config.yaml");
//   ...
//   tx.stop();
class ExtSensorIoTx {
public:
    static ExtSensorIoTx& instance();

    // Starts every sensor configured in the file (a sensor is skipped
    // when its section is absent or has `enabled: false`). Returns
    // false when nothing could start.
    bool start(const std::string& config_path = "config/config.yaml");
    void stop();

private:
    ExtSensorIoTx() = default;

    UwbTransmitter uwb_;
    bool           uwb_running_ = false;
};

} // namespace kist
