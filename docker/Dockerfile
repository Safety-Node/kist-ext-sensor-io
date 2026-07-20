# kist-ext-sensor-io — all sensor-I/O deps in one image (both Tx and Rx
# roles). No GPU needed. The repo itself is bind-mounted at run time
# (see run.sh) and built inside; this image only provides dependencies.
#
#   deps by role:
#     Tx (device machine): librealsense2 (RealSense), x264 (H.264 encode),
#                          UWB serial via /dev (no lib)
#     Rx (workstation):    libavcodec/util/swscale (H.264 decode), OpenCV
#     both:                unitree_sdk2 (vendored, built from thirdparty),
#                          yaml-cpp
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ─── build tools + light deps ─────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ─── camera codec + view deps ─────────────────────────────────────────────────
# x264 (Tx color encode), libav (Rx color decode), OpenCV (visual probes).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libx264-dev \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# ─── librealsense2 (Intel apt repo) ───────────────────────────────────────────
# Tx-side capture. Runtime needs USB access to the camera (see run.sh).
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
      | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dev \
    librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/kist-ext-sensor-io

CMD ["/bin/bash"]
