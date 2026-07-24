# kist-ext-sensor-io

External (attached) sensor I/O for the KIST G1 stack

Sensors: UWB (Decawave DWM1001-dev), RealSense camera(D435i)

## Architecture

[![Architecture](docs/kist-ext-sensor-io.svg)](docs/kist-ext-sensor-io.svg)

## Dependencies

The Docker image (recommended path below) installs all of these for you. The
table + host-install steps are only for building **without** Docker (Ubuntu).

| Component | Version | Role |
|---|---|---|
| `unitree_sdk2` | `21d0a3b` | both — DDS client + ROS2 `PoseStamped` IDL; bundles the CycloneDDS 0.10.2 runtime libs |
| CycloneDDS + CycloneDDS-CXX | 0.10.2 | build-time — `idlc`/`idlcxx` codegen for the custom compressed-frame types (pinned to match the SDK's bundled `libddscxx`) |
| `librealsense2` | v2.58.1 | Tx — RealSense capture |
| x264 | distro | Tx — H.264 color encode |
| FFmpeg / libav | distro | Rx — H.264 color decode |
| OpenCV | distro | Rx — viewer probe only (`test_realsense_receiver_viewer`) |
| `yaml-cpp` | distro | both — config parsing |

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

Ubuntu. Mirrors what the Dockerfile does; run from the repository root.

**1. apt packages** (build tools, codecs, OpenCV, yaml-cpp, librealsense build deps)
```bash
sudo apt update && sudo apt install -y \
    build-essential cmake git pkg-config \
    libyaml-cpp-dev \
    libx264-dev libavcodec-dev libavutil-dev libswscale-dev libopencv-dev \
    libusb-1.0-0-dev libudev-dev libssl-dev
```

**2. CycloneDDS + CycloneDDS-CXX 0.10.2** — the `idlc` toolchain, into `/opt/cyclonedds`
(pinned to match the SDK's bundled `libddscxx`; we use only the codegen tools):
```bash
git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds
cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds/build --target install -j"$(nproc)"

git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx
cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds-cxx/build --target install -j"$(nproc)"

export PATH=/opt/cyclonedds/bin:$PATH      # idlc on PATH for the build below
```

**3. librealsense2 v2.58.1** (from source, library only — no examples/CUDA):
```bash
git clone --depth 1 -b v2.58.1 https://github.com/realsenseai/librealsense.git /tmp/librealsense
cmake -S /tmp/librealsense -B /tmp/librealsense/build -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_WITH_CUDA=false
sudo cmake --build /tmp/librealsense/build --target install -j"$(nproc)" && sudo ldconfig
```

**4. unitree_sdk2** (vendored under `thirdparty/`, pinned):
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
git -C thirdparty/unitree_sdk2 checkout 21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
```

**5. Build:**
```bash
cmake -B build && cmake --build build -j"$(nproc)"
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
