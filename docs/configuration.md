# Configuration

All binaries read `config/config.yaml`. Edit it per deployment (on Docker, edit
it inside the container — the image bakes it in).

## `unitree`

| Key | Default | Meaning |
|---|---|---|
| `domain_id` | `0` | DDS domain — must match on Tx and Rx |
| `dds_config` | `config/cyclonedds.xml` | DDS transport config (XML) — the NIC and socket tuning live there (table below). A pre-set `CYCLONEDDS_URI` env wins over this path |

## `uwb`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | run the UWB Tx/Rx |
| `serial_port` | `/dev/uwb` | DWM dongle node (or the raw node, e.g. `/dev/ttyACM0`) |
| `serial_baud` | `115200` | DWM UART baud |
| `frame_id` | `map` | frame_id stamped on published fixes |

## `realsense_cameras` + `realsense_defaults`

Cameras are a **list** — one entry per physical RealSense. Shared settings live
in `realsense_defaults`; each `realsense_cameras` entry adds `name` + `serial`
and may override any default. A runner spawns one Transmitter/Receiver per entry
(each with its own threads), and every camera's DDS topics are namespaced by its
`name`. Consumers subscribe to whichever camera(s) they want by name.

### `realsense_cameras` — per-camera entries

| Key | Default | Meaning |
|---|---|---|
| `name` | (required) | camera name. Namespaces the DDS topics `rt/kist/camera/<name>/color/h264` and `.../depth/rvl`, and defaults the frame_ids (`<name>_depth` / `<name>_color`). Tx and Rx agree by this name. |
| `serial` | `""` → first device | RealSense serial to open (from `rs-enumerate-devices`). Set it to pick a specific camera when several are attached. |
| `enabled` | `true` | run this camera; `false` skips it on **both** Tx and Rx (handy to disable one without deleting the entry) |

Any `realsense_defaults` field may also appear in an entry to override it for
that one camera (e.g. a wrist D405 at 640×480 while the head runs 1280×720).

```yaml
realsense_cameras:
  - name: head
    serial: "938422073271"
  - name: left_wrist
    serial: "260422272337"
    color_fps: 15            # override just this camera
  - name: right_wrist
    serial: "260322270228"
    enabled: false           # temporarily off
```

### `realsense_defaults` — shared capture settings

| Key | Default | Meaning |
|---|---|---|
| `depth_width` / `depth_height` / `depth_fps` | `640` / `480` / `30` | depth (Z16) stream mode |
| `color_width` / `color_height` / `color_fps` | `640` / `480` / `30` | color (BGR8) stream mode |
| `align_to_color` | `true` | reproject depth into the color frame (CPU-heavy) |
| `depth_frame_id` / `color_frame_id` | `<name>_depth` / `<name>_color` | frame ids (default from the camera name) |

Higher resolutions (e.g. 1280×720) are much heavier. With **several cameras on one
machine**, watch USB bandwidth (a shared hub) and — on a Jetson — CPU for `align` +
H.264 encode; drop `color_fps` or `align_to_color` per camera if frames stall.

### `realsense_defaults.spatial_filter` — depth edge-preserving filter

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | apply the filter |
| `magnitude` | `2` | filter passes (1–5) |
| `smooth_alpha` | `0.5` | smoothing weight (0–1) |
| `smooth_delta` | `20.0` | depth-discontinuity threshold (mm) |

### `realsense_defaults` — H.264 color encoder (x264)

| Key | Default | Meaning |
|---|---|---|
| `color_bitrate_kbps` | `4000` | target bitrate |
| `color_keyframe_interval` | `30` | keyframe (IDR) interval, in frames |
| `color_preset` | `ultrafast` | x264 speed/quality preset |
| `color_tune` | `zerolatency` | x264 tune |
| `color_profile` | `baseline` | H.264 profile |

## `mics` + `mic_defaults`

Microphones are a **list**, same shape as the cameras: shared settings live
in `mic_defaults`; each `mics` entry adds `name` + `device` and may override
any default. A runner spawns one Transmitter/Receiver per entry, and every
mic's DDS topic is namespaced by its `name`. Capture is S16_LE interleaved
PCM at the given rate/channels — a mode the hardware can't do fails loudly
at start (`arecord --dump-hw-params` lists what it can do).

### `mics` — per-mic entries

| Key | Default | Meaning |
|---|---|---|
| `name` | (required) | mic name. Namespaces the DDS topic `rt/kist/mic/<name>/audio` and defaults the frame_id. Tx and Rx agree by this name. |
| `device` | (required) | ALSA card name to match (`arecord -l` lists them — card numbers move between boots, names don't), or a literal PCM name (`hw:3,0`) |
| `enabled` | `true` | run this mic; `false` skips it on both Tx and Rx |

Any `mic_defaults` field may also appear in an entry to override it for that
one mic.

```yaml
mics:
  - name: array
    device: L16K6Ch        # reSpeaker Flex XVF3800 (fixed 16k/6ch)
    sample_rate: 16000
    channels: 6
  - name: uno
    device: UNO            # ESI NEVA UNO
    sample_rate: 48000
    channels: 2
```

### `mic_defaults` — shared capture settings

| Key | Default | Meaning |
|---|---|---|
| `sample_rate` | `16000` | capture rate (Hz) |
| `channels` | `1` | interleaved channel count |
| `chunk_ms` | `100` | PCM per DDS message (100ms -> 10 chunks/s) |
| `frame_id` | `<name>` | frame_id stamped on published chunks |

## `cyclonedds.xml`

The network interface and the DDS transport tuning live here — **not in
config.yaml**.

| Element | Value | Meaning |
|---|---|---|
| `General/Interfaces/NetworkInterface name` | `lo` | default is same-machine testing; for deployment set the LAN NIC (e.g. `eth0`, `eno2`) here |
| `General/FragmentSize` / `MaxMessageSize` / `MaxRexmitMessageSize` | `1344 B` / `1400 B` / `1400 B` | UDP datagrams capped at one MTU |
| `Internal/SocketSendBufferSize min` | `16MiB` | UDP send buffer size — needs the kernel limit raised once per machine (README, System setup) |
| `Internal/SocketReceiveBufferSize min` | `16MiB` | UDP receive buffer size — same kernel limit |
