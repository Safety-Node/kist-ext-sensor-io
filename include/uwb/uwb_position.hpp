#pragma once

#include <cstdint>

namespace kist {

// UWB positions on the wire. rt/ prefix keeps the topic visible to
// ROS2 tools as /kist/uwb/pose; the type is the SDK-vendored ROS2 IDL
// (PoseStamped), so subscribers need no custom IDL.
inline constexpr const char* kUwbPoseTopic = "rt/kist/uwb/pose";

// One UWB position fix — the output of UwbReceiver (Rx side).
//
// Producer contract (UwbTransmitter, Tx side):
//   - valid fixes only (DWM quality > 0), no placeholder publish
//   - UWB local frame (anchor coordinates), DWM does trilateration
//     internally; there is no heading source
//   - goes silent when the tag has no fix or the serial link dies ->
//     the consumer's buffer stales (empty-buffer principle)
struct UwbPosition {
    // Publisher timestamp (system clock at publish).
    int64_t stamp_ns = 0;

    // Position in the UWB local frame (m).
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

} // namespace kist
