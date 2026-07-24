# Embedding a Receiver

A sensor spans two sides:

- **Transmitter** — device side (where the sensor is plugged in): reads the
  device → encodes → publishes over DDS.
- **Receiver** — consumer side (your app): subscribes → decodes → writes the
  latest frame/fix into a buffer you poll.

`start()` spins up the background receive/decode threads and returns; your code
then polls the buffer for the latest value and calls `stop()` when done (keep the
process alive in between). Tx and Rx find each other over DDS only if they share
the same `domain_id` and sit on the same network (`network_interface`).

## Link the receiver

```cmake
add_subdirectory(kist-ext-sensor-io)
target_link_libraries(your_app PRIVATE realsense_receiver)   # and/or uwb_receiver
```

## RealSense

```cpp
#include "system/realsense_receiver.hpp"

kist::RealsenseReceiver rx;
if (!rx.start(domain_id, network_interface))
    return 1;

// poll from any thread; empty buffer = no live frame (1s watchdog)
auto color = rx.color().GetData();   // decoded BGR8
auto depth = rx.depth().GetData();   // decoded Z16

rx.stop();
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
