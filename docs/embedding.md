# Embedding a Receiver

A sensor spans two sides:

- **Transmitter** — device side (where the sensor is plugged in)
- **Receiver** — consumer side (your app)

## Link the receiver

```cmake
add_subdirectory(kist-ext-sensor-io)
target_link_libraries(your_app PRIVATE realsense_receiver)   # and/or uwb_receiver
```

## UWB

Same shape — `rx.fix()` for the latest position, or a hook that fires on each new
fix instead of polling (runs on the DDS thread — keep it cheap):

```cpp
#include "common/dds_config.hpp"
#include "system/uwb_receiver.hpp"

kist::UwbReceiver rx;
kist::apply_dds_config(root);   // routes config/cyclonedds.xml (the NIC lives there)
rx.start(domain_id, "");        // empty interface — a non-empty one disables the XML

auto fix = rx.fix().GetDataWithTime();
// or, callback instead of polling:
rx.set_on_position([](const kist::UwbPosition& p) { /* forward p */ });

rx.stop();
```


## RealSense

One `RealsenseReceiver` per camera — `name` selects which camera's topics to
subscribe to (`rt/kist/camera/<name>/...`), so it must match a `name` the
transmitter published. Start one per camera you want:

```cpp
#include "common/dds_config.hpp"
#include "system/realsense_receiver.hpp"

kist::RealsenseReceiver rx;
kist::apply_dds_config(root);   // routes config/cyclonedds.xml (the NIC lives there)
if (!rx.start(domain_id, "", "head"))                  // camera name
    return 1;

// poll from any thread; empty buffer = no live frame (1s watchdog)
auto color = rx.color().GetData();   // decoded BGR8
auto depth = rx.depth().GetData();   // decoded Z16

rx.stop();
```

The names are in `config/config.yaml` (`realsense_cameras[].name`);
`kist::camera_names_from_yaml(root)` returns them if you'd rather not hardcode.

## MicReceiver

One `MicReceiver` per microphone — `name` selects the topic
(`rt/kist/mic/<name>/audio`). Chunks are self-describing (rate / channels /
format ride along). The buffer is latest-wins, so a consumer that needs
EVERY chunk (contiguous audio) must tap the hook — set it before `start()`:

```cpp
#include "common/dds_config.hpp"
#include "system/mic_receiver.hpp"

kist::MicReceiver rx;
kist::apply_dds_config(root);
rx.set_on_chunk([](const kist::AudioChunk& c) {
    // runs on the DDS receive thread — keep cheap (copy out, signal, return)
    // c.data = interleaved PCM (c.format, c.sample_rate, c.channels)
});
if (!rx.start(domain_id, "", "array"))                 // mic name
    return 1;

// or poll liveness/latest from any thread (not a contiguous stream)
auto chunk = rx.chunk().GetData();

rx.stop();
```

The names are in `config/config.yaml` (`mics[].name`);
`kist::mic_names_from_yaml(root)` returns them.