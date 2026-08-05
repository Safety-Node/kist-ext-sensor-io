#pragma once

#include "system/realsense_transmitter.hpp"
#include "system/uwb_transmitter.hpp"
#include "realsense/realsense_config.hpp"   // CameraSpec

#include <memory>
#include <string>
#include <vector>

namespace kist {

// Whole-device Tx assembly — everything this machine publishes behind one
// start/stop: the UWB transmitter (when settings are given) plus one
// RealsenseTransmitter per camera. This is what the kist_ext_sensor_io
// runner drives; the per-sensor test_* runners remain for isolating one
// sensor. Owns no thread of its own; config parsing stays in the runner.
class ExtSensorIoTx {
public:
    struct UwbSettings {
        std::string serial_port;
        int         serial_baud = 115200;
        std::string frame_id    = "map";
    };

    struct Camera {
        std::string name;
        std::unique_ptr<RealsenseTransmitter> tx;
    };

    // uwb == nullptr disables the UWB side. A camera that fails to start is
    // skipped (reported on stderr), not fatal; returns false only when
    // nothing at all started.
    bool start(int domain_id, const std::string& network_interface,
               const UwbSettings* uwb, const std::vector<CameraSpec>& cameras);
    void stop();

    // For the runner's 1 Hz report.
    const std::vector<Camera>& cameras() const { return cams_; }
    UwbTransmitter* uwb() { return uwb_ ? uwb_.get() : nullptr; }

private:
    std::vector<Camera>             cams_;
    std::unique_ptr<UwbTransmitter> uwb_;
    bool                            running_ = false;
};

} // namespace kist
