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
#include "system/uwb_receiver.hpp"

kist::UwbReceiver rx;
rx.start(domain_id, network_interface);

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
#include "system/realsense_receiver.hpp"

kist::RealsenseReceiver rx;
if (!rx.start(domain_id, network_interface, "head"))   // camera name
    return 1;

// poll from any thread; empty buffer = no live frame (1s watchdog)
auto color = rx.color().GetData();   // decoded BGR8
auto depth = rx.depth().GetData();   // decoded Z16

rx.stop();
```

The names are in `config/config.yaml` (`realsense_cameras[].name`);
`kist::camera_names_from_yaml(root)` returns them if you'd rather not hardcode.