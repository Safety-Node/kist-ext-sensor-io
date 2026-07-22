#pragma once

namespace kist {

// One parsed DWM position line — the Tx-side raw sample, produced by
// the serial reader and consumed by the publisher thread (sample_buf).
//
// Relation to UwbPosition (the wire/Rx contract): UwbSample still
// carries the DWM quality factor; the publisher only forwards samples
// the parser already validated (quality > 0), so quality never crosses
// the wire — "it was published" is the validity statement.
struct UwbSample {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // UWB local frame (m)
    int   quality = 0;                   // DWM quality factor, 1-100
};

} // namespace kist
