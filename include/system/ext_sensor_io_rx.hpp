#pragma once

#include "system/realsense_receiver.hpp"
#include "system/uwb_receiver.hpp"

#include <memory>
#include <string>
#include <vector>

namespace kist {

// Whole-device Rx assembly — the mirror of ExtSensorIoTx: the UWB receiver
// (when enabled) plus one RealsenseReceiver per camera name, behind one
// start/stop. This is what the kist_ext_sensor_io runner drives; consumers
// embedding single sensors keep using the per-sensor assemblies directly.
class ExtSensorIoRx {
public:
    struct Camera {
        std::string name;
        std::unique_ptr<RealsenseReceiver> rx;
    };

    // A camera that fails to start is skipped (reported), not fatal;
    // returns false only when nothing at all started.
    bool start(int domain_id, const std::string& network_interface,
               bool uwb_enabled, const std::vector<std::string>& camera_names);
    void stop();

    // For the runner's 1 Hz report.
    const std::vector<Camera>& cameras() const { return cams_; }
    UwbReceiver* uwb() { return uwb_ ? uwb_.get() : nullptr; }

private:
    std::vector<Camera>          cams_;
    std::unique_ptr<UwbReceiver> uwb_;
    bool                         running_ = false;
};

} // namespace kist
