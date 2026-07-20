# kist-ext-sensor-io

External (attached) sensor I/O for the KIST G1 stack, without ROS2: per sensor a DDS publisher (Tx, runs where the device is plugged in) and a reader library (Rx, embedded by consumers). Unitree built-in sensors are NOT here — they live with their consumers (e.g. kist-navigation-planner's `unitree_*_reader`).

Sensors: UWB (Decawave DWM). Planned: camera, mic.

## Architecture

[![Architecture](docs/kist-ext-sensor-io.svg)](docs/kist-ext-sensor-io.svg)

## Dependencies

| Package | Purpose |
|---|---|
| `unitree_sdk2` | DDS client library + ROS2 IDL types (PoseStamped) |
| `yaml-cpp` | YAML config parser |

## Installation

### Clone Repository

```bash
git clone https://github.com/Safety-Node/kist-ext-sensor-io.git
cd kist-ext-sensor-io
```

All following steps run from the repository root.

### Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
```

### Install yaml-cpp

```bash
sudo apt install libyaml-cpp-dev
```

### UWB udev rule (target machine)

The DWM dongle (DWM1001-DEV: SEGGER J-Link OB, VID=1366 PID=0105) must
appear as `/dev/uwb`:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", SYMLINK+="uwb", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-uwb.rules
sudo udevadm control --reload && sudo udevadm trigger
```

`MODE="0666"` lets the daemon run as a normal user (no dialout
membership needed).

Verify the IDs with `udevadm info /dev/ttyACM0 | grep ID_` if the dongle
differs.

## Build

```bash
cmake -B build && cmake --build build
```

## Run

### Offline checks (no hardware)

```bash
./build/test_dwm_parse       # DWM lec line parser
./build/test_uwb_loopback    # transmitter -> receiver over lo (full DDS roundtrip)
```

### UWB transmitter daemon (on the machine with the dongle)

```bash
./build/uwb_transmitter        # reads config/config.yaml
```

Prints a once-per-second status line (`rate= XHz pos=(...) quality=..`,
or `no fix`). Publishes `rt/kist/uwb/pose` (`geometry_msgs/PoseStamped`)
— visible to ROS2 tools as `/kist/uwb/pose`. Valid fixes only; the topic
goes silent when the tag has no fix.

## Usage

Embedding the receiver as a C++ library:

```cmake
add_subdirectory(kist-ext-sensor-io)
target_link_libraries(your_app PRIVATE uwb_io)
```

```cpp
#include "uwb/uwb_receiver.hpp"

auto& uwb = kist::UwbReceiver::instance();
if (!uwb.start("config/config.yaml"))     // reads unitree.domain_id / network_interface
    return 1;

// consume by polling from any thread; empty buffer = no fix for 1s
auto fix = uwb.uwb_buf.GetDataWithTime();
if (fix.HasData())
    use(*fix.data);

uwb.stop();
```

Optional, before `start()`: react to every fix as it arrives (runs on the
DDS receive thread — keep it cheap). This is how an integrating wrapper
forwards fixes into another module's passive buffer:

```cpp
uwb.set_on_position([](const kist::UwbPosition& p) { /* forward p */ });
```

`start(domain_id, network_interface)` is also available when the caller
already loaded its own config.
