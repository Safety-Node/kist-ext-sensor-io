# Configuration

All binaries read `config/config.yaml`. Edit it per deployment (on Docker, edit
it inside the container — the image bakes it in).

## `unitree`

| Key | Default | Meaning |
|---|---|---|
| `domain_id` | `0` | DDS domain — must match on Tx and Rx |
| `network_interface` | `lo` | NIC that DDS binds to. `lo` for same-machine; the LAN interface (e.g. `eth0`, `eno2`) for two machines. Names differ per host (`ip -brief addr`); both ends must share a subnet |

## `uwb`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | run the UWB Tx/Rx |
| `serial_port` | `/dev/uwb` | DWM dongle node (or the raw node, e.g. `/dev/ttyACM0`) |
| `serial_baud` | `115200` | DWM UART baud |
| `frame_id` | `map` | frame_id stamped on published fixes |

## `camera`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | run the camera Tx/Rx |
| `depth_width` / `depth_height` / `depth_fps` | `640` / `480` / `30` | depth (Z16) stream mode |
| `color_width` / `color_height` / `color_fps` | `640` / `480` / `30` | color (BGR8) stream mode |
| `align_to_color` | `true` | reproject depth into the color frame (CPU-heavy) |
| `depth_frame_id` / `color_frame_id` | `camera_depth` / `camera_color` | frame ids |

Higher resolutions (e.g. 1280×720) are much heavier — more CPU for `align` + H.264
encode, and larger depth frames on the wire.

### `camera.spatial_filter` — depth edge-preserving filter

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | apply the filter |
| `magnitude` | `2` | filter passes (1–5) |
| `smooth_alpha` | `0.5` | smoothing weight (0–1) |
| `smooth_delta` | `20.0` | depth-discontinuity threshold (mm) |

### `camera` — H.264 color encoder (x264)

| Key | Default | Meaning |
|---|---|---|
| `color_bitrate_kbps` | `4000` | target bitrate |
| `color_keyframe_interval` | `30` | keyframe (IDR) interval, in frames |
| `color_preset` | `ultrafast` | x264 speed/quality preset |
| `color_tune` | `zerolatency` | x264 tune |
| `color_profile` | `baseline` | H.264 profile |
