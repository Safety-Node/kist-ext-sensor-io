# kist-ext-sensor-io

External (attached) sensor I/O for the KIST G1 stack, without ROS2: per sensor a DDS publisher (Tx, runs where the device is plugged in) and a reader library (Rx, embedded by consumers). Unitree built-in sensors are NOT here — they live with their consumers (e.g. kist-navigation-planner's `unitree_*_reader`).

Sensors: UWB (Decawave DWM), RealSense camera. Planned: mic.

## Architecture

[![Architecture](docs/kist-ext-sensor-io.svg)](docs/kist-ext-sensor-io.svg)

## Dependencies

All of these are provided by the Docker image (the recommended path below);
the table is for host builds:

| Package | Purpose |
|---|---|
| `unitree_sdk2` | DDS client + ROS2 IDL types (PoseStamped) |
| CycloneDDS `idlc` 0.10.2 | codegen for the custom compressed-frame DDS types |
| `librealsense2` | RealSense capture (camera Tx) |
| `x264` / `libavcodec` | H.264 color encode (Tx) / decode (Rx) |
| OpenCV | viewer probe (Rx) |
| `yaml-cpp` | YAML config parser |

## Quick start (Docker)

The image is self-contained — it clones `unitree_sdk2` (pinned) and builds the
whole repo at image-build time. No host deps, no SDK vendoring, no bind mount.

```bash
git clone https://github.com/Safety-Node/kist-ext-sensor-io.git
cd kist-ext-sensor-io
docker build -f docker/Dockerfile -t kist-ext-sensor-io .
docker/run.sh            # opens a shell; prebuilt binaries live under build/
```

`run.sh` wires `--network host` (DDS discovery), `--privileged -v /dev`
(RealSense USB + `/dev/uwb`), and X11 (the viewer). For iterative dev, bind-mount
your working copy over the baked source — see the comments in `run.sh`.

## Host build (without Docker)

Install the dependencies above, then — from the repository root:

```bash
# vendored SDK (Docker does this for you; needed only for host builds)
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2

cmake -B build && cmake --build build
```

## UWB device setup (Tx machine)

The DWM dongle (DWM1001-DEV: SEGGER J-Link OB, VID=1366 PID=0105) must appear
as `/dev/uwb` on the host (Docker passes it through via `-v /dev`):

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", SYMLINK+="uwb", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-uwb.rules
sudo udevadm control --reload && sudo udevadm trigger
```

`MODE="0666"` lets it run as a normal user. Verify the IDs with
`udevadm info /dev/ttyACM0 | grep ID_` if the dongle differs.

## Usage

Each sensor is a pair of own-no-thread **assemblies** — a `*Transmitter`
(device side: read → encode → publish) and a `*Receiver` (consumer side:
subscribe → decode → buffer). Link the assembly you need and drive it
in-process; config parsing lives in the caller.

### Embed the Rx side (consumer app)

```cmake
add_subdirectory(kist-ext-sensor-io)
target_link_libraries(your_app PRIVATE uwb_receiver)   # and/or realsense_receiver
```

```cpp
#include "system/uwb_receiver.hpp"

kist::UwbReceiver rx;
if (!rx.start(domain_id, network_interface))
    return 1;

// poll from any thread; empty buffer = no fix for 1s (watchdog)
auto fix = rx.fix().GetDataWithTime();
if (fix.HasData())
    use(*fix.data);

rx.stop();
```

RealSense is identical in shape — link `realsense_receiver`, then read
`rx.color()` / `rx.depth()` for the decoded frames.

Optional, before `start()`: react to every fix on arrival (runs on the DDS
receive thread — keep it cheap):

```cpp
rx.set_on_position([](const kist::UwbPosition& p) { /* forward p */ });
```

### Run standalone (device / testing)

Each assembly has a runner that reads `config/config.yaml`:

```bash
./build/test_uwb_transmitter           # device:   DWM dongle -> DDS
./build/test_uwb_receiver              # consumer: prints received fixes

./build/test_realsense_transmitter     # device:   D435i -> DDS
./build/test_realsense_receiver        # consumer: prints fps
./build/test_realsense_receiver_viewer # consumer: shows color | depth
```
