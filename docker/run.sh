#!/bin/bash
# Launch (or re-attach to) a persistent named container.
# Reuse across sessions so builds and caches survive until you explicitly
# `docker rm kist-ext-sensor-io`.
#
# Wired for BOTH roles from one image:
#   --network host                DDS discovery/multicast (Tx<->Rx)
#   --privileged -v /dev:/dev     RealSense USB + UWB serial (/dev/uwb)  [Tx]
#   DISPLAY + /tmp/.X11-unix       OpenCV live windows                   [Rx]

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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
        -v "${REPO_DIR}:/workspace/kist-ext-sensor-io" \
        -w /workspace/kist-ext-sensor-io \
        kist-ext-sensor-io
fi
