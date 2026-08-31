#!/bin/bash
# Launch (or re-attach to) a persistent named container.
# Reuse across sessions so builds and caches survive until you explicitly
# `docker rm kist-ext-sensor-io`.
#
# The image is self-contained (source baked in + built); no source mount.
# Wired for BOTH roles from one image:
#   --network host                DDS discovery/multicast (Tx<->Rx)
#   --privileged -v /dev:/dev     RealSense USB + UWB serial (/dev/uwb)  [Tx]
#   DISPLAY + /tmp/.X11-unix       OpenCV live windows                   [Rx]
#
# No GPU flag on either arch: this repo uses no CUDA (librealsense built with
# CUDA off, no TensorRT), so nothing to pass — works as-is on x86 and on the
# Jetson/Orin (aarch64). --network host also sidesteps the Orin's bridge-NAT
# gap (docker there runs with "iptables": false). If you ever compose with a
# CUDA image, add the platform-appropriate GPU flag here.
#
# Iterative dev: add  -v "$(pwd)":/workspace/kist-ext-sensor-io  to shadow the
# baked source with your working copy (then rebuild build/ inside).

set -e

CONTAINER=kist-ext-sensor-io

if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    docker exec -it "${CONTAINER}" /bin/bash
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    docker start -ai "${CONTAINER}"
else
    xhost +local:root >/dev/null 2>&1 || true
    docker run -it \
        --name "${CONTAINER}" \
        --network host \
        --privileged \
        -v /dev:/dev \
        -e DISPLAY="${DISPLAY}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -w /workspace/kist-ext-sensor-io \
        kist-ext-sensor-io
fi
