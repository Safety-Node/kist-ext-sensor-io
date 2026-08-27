# kist-ext-sensor-io — all sensor-I/O deps in one image (both Tx and Rx
# roles). Base matched to kist-gearsonic-inference so the two compose into
# one orchestrated image later. The repo is copied in, unitree_sdk2 is cloned
# (pinned) inside, and it's all built at image-build time — so the image is
# self-contained: no host-side vendoring, no bind mount, no in-container setup.
# `docker run` gives ready binaries. For iterative dev you can still shadow the
# baked source with a bind mount (see run.sh comments).
#
#   deps by role:
#     Tx (device machine): librealsense2 (RealSense), x264 (H.264 encode),
#                          UWB serial via /dev (no lib)
#     Rx (workstation):    libavcodec/util/swscale (H.264 decode), OpenCV
#     both:                unitree_sdk2 (cloned in-image, pinned),
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
    libasound2-dev \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# ─── idlc toolchain (custom DDS IDL types) ────────────────────────────────────
# CycloneDDS 0.10.2 pinned to match unitree_sdk2's bundled libddscxx 0.10.2,
# so generated type headers are ABI/format-compatible with the SDK's runtime
# libs. CycloneDDS-CXX provides the C++ backend (idlcxx) that ros-humble's
# idlc lacked. Installed to /opt so it never shadows the SDK's own libs — we
# use only the idlc/idlcxx BUILD tools; linking still uses the vendored .so.
# ENABLE_TYPE_DISCOVERY=OFF: the 0.10.x XTypes parser segfaults
# (ddsi_xt_type_init_impl "invalid type object") on the TypeObject a modern
# peer advertises in discovery — e.g. the python `cyclonedds` 11.x reader in
# kist-vla-inference subscribing to the camera stream. With type discovery
# off the core neither advertises nor parses TypeObjects and endpoints match
# by type name (how the unitree bus already interops). TOPIC_DISCOVERY
# depends on TYPE_DISCOVERY, so it goes off together.
ARG CYCLONEDDS_VERSION=0.10.2
RUN git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds && \
    cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_TYPE_DISCOVERY=OFF -DENABLE_TOPIC_DISCOVERY=OFF && \
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
# Make the runtime LOAD this type-discovery-less core: the binaries' RUNPATH
# points at unitree_sdk2's vendored libddsc/libddscxx (built WITH type
# discovery — that copy is what crashed), and LD_LIBRARY_PATH takes
# precedence over DT_RUNPATH. Same 0.10.2, so the ABI the SDK pin exists
# for is preserved.
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib
# Enlarge the DDS socket buffers (big depth/color frames overflow the default UDP
# buffers -> receiver drops samples). config/cyclonedds.xml lands here via COPY.
# See that file re: the host net.core.[rw]mem_max cap under --network host.
ENV CYCLONEDDS_URI=file:///workspace/kist-ext-sensor-io/config/cyclonedds.xml

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

# ─── vendored dep: unitree_sdk2 (cloned + pinned) ─────────────────────────────
# Pinned to the commit the repo was validated against — its bundled libddscxx
# 0.10.2 matches the idlc built above, so generated types stay ABI-compatible.
# Cloned here (not host-vendored, not COPY'd — see .dockerignore) so the image
# is fully self-sufficient. Placed before COPY so source edits don't re-clone.
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2 && \
    git -C thirdparty/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}"

# ─── source + build ───────────────────────────────────────────────────────────
# Bake the repo in (build context = repo root, minus build/ .git & the SDK via
# .dockerignore). Deps + SDK layers above stay cached; only this re-runs on a
# source change.
COPY . .
RUN cmake -B build && cmake --build build -j"$(nproc)"

CMD ["/bin/bash"]
