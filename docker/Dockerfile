# kist-ext-sensor-io — all sensor-I/O deps in one image (both Tx and Rx
# roles). Base matched to kist-gearsonic-inference so the two compose into
# one orchestrated image later. The repo is bind-mounted at run time
# (see run.sh) and built inside; this image only provides dependencies.
#
#   deps by role:
#     Tx (device machine): librealsense2 (RealSense), x264 (H.264 encode),
#                          UWB serial via /dev (no lib)
#     Rx (workstation):    libavcodec/util/swscale (H.264 decode), OpenCV
#     both:                unitree_sdk2 (vendored, built from thirdparty),
#                          yaml-cpp, idlc toolchain (custom DDS IDL types)
FROM nvcr.io/nvidia/tensorrt:24.12-py3

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

# ─── idlc toolchain (custom DDS IDL types) ────────────────────────────────────
# CycloneDDS 0.10.2 pinned to match unitree_sdk2's bundled libddscxx 0.10.2,
# so generated type headers are ABI/format-compatible with the SDK's runtime
# libs. CycloneDDS-CXX provides the C++ backend (idlcxx) that ros-humble's
# idlc lacked. Installed to /opt so it never shadows the SDK's own libs — we
# use only the idlc/idlcxx BUILD tools; linking still uses the vendored .so.
ARG CYCLONEDDS_VERSION=0.10.2
RUN git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds && \
    cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/cyclonedds/build --target install -j "$(nproc)" && \
    git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx && \
    cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/cyclonedds-cxx/build --target install -j "$(nproc)" && \
    rm -rf /tmp/cyclonedds /tmp/cyclonedds-cxx
ENV PATH=/opt/cyclonedds/bin:${PATH}
ENV CYCLONEDDS_HOME=/opt/cyclonedds

# ─── librealsense2 (built from source) ────────────────────────────────────────
# Tx-side capture. Source-built (not Intel's apt repo, whose noble signing
# key is broken) so it's base/arch-agnostic and matches the host's version.
# Library only: BUILD_EXAMPLES/GRAPHICAL off (headless container). The
# kernel-module patch + udev scripts are NOT run here — those are host-level
# (done on the host); the container reaches the camera via the host kernel +
# USB passthrough (see run.sh --privileged / -v /dev).
ARG LIBRS_VERSION=v2.58.1
RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0-dev libudev-dev libssl-dev && \
    git clone --depth 1 --branch "${LIBRS_VERSION}" \
        https://github.com/realsenseai/librealsense.git /tmp/librealsense && \
    cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false \
        -DBUILD_WITH_CUDA=false && \
    cmake --build /tmp/librealsense/build --target install -j "$(nproc)" && \
    ldconfig && \
    rm -rf /tmp/librealsense /var/lib/apt/lists/*

WORKDIR /workspace/kist-ext-sensor-io

CMD ["/bin/bash"]
