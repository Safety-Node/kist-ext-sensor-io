cmake_minimum_required(VERSION 3.18)
project(kist_ext_sensor_io CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

# ─── thirdparty: unitree_sdk2 ─────────────────────────────────────────────────
# Guarded so an embedding super-project that already provides the target
# (e.g. via kist-navigation-planner or kist-gearsonic-inference)
# composes without a collision.
if(NOT TARGET unitree_sdk2)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
    add_subdirectory(thirdparty/unitree_sdk2)
endif()

# ─── DDS IDL codegen (idlc, cxx backend) ──────────────────────────────────────
# Custom compressed-frame types generated from idl/*.idl. idlc + the C++
# backend come from /opt/cyclonedds in the Docker image (pinned 0.10.2 to
# match the SDK's bundled ddscxx). Generated header lands in build/idl_gen.
find_program(CYCLONE_IDLC idlc HINTS /opt/cyclonedds/bin)
if(CYCLONE_IDLC)
    set(KIST_IDL_GEN ${CMAKE_BINARY_DIR}/idl_gen)
    file(MAKE_DIRECTORY ${KIST_IDL_GEN})
    # idlc (cxx backend) emits <name>.hpp AND <name>.cpp into its working
    # directory (it ignores -o here), so run it IN the gen dir. The .cpp
    # holds the type descriptors and must be compiled + linked.
    add_custom_command(
        OUTPUT  ${KIST_IDL_GEN}/kist_camera_frames.hpp
                ${KIST_IDL_GEN}/kist_camera_frames.cpp
        COMMAND ${CYCLONE_IDLC} -l cxx
                ${CMAKE_CURRENT_SOURCE_DIR}/idl/kist_camera_frames.idl
        WORKING_DIRECTORY ${KIST_IDL_GEN}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/idl/kist_camera_frames.idl
        COMMENT "idlc: kist_camera_frames.idl -> C++ DDS types")
    add_library(kist_idl STATIC ${KIST_IDL_GEN}/kist_camera_frames.cpp)
    target_include_directories(kist_idl PUBLIC ${KIST_IDL_GEN})
    target_link_libraries(kist_idl PUBLIC unitree_sdk2)  # ddscxx headers/libs
else()
    message(STATUS "idlc not found (/opt/cyclonedds/bin) — IDL codegen unavailable")
endif()

# ─── uwb leaf units ───────────────────────────────────────────────────────────
# One class per thread: UwbSerialReader (serial -> samples) + UwbPublisher
# (samples -> DDS) on the Tx side, UwbSubscriber (DDS -> uwb_buf) on the Rx.
# The Tx/Rx assemblies live in system/ (uwb_transmitter, uwb_receiver).
add_library(uwb_io STATIC
    src/uwb/transmitter/uwb_serial_reader.cpp
    src/uwb/transmitter/uwb_publisher.cpp
    src/uwb/receiver/uwb_subscriber.cpp
)
target_include_directories(uwb_io PUBLIC include)
target_link_libraries(uwb_io PUBLIC unitree_sdk2)

# ─── realsense camera (ported from depth-cam-stream-codec, ROS2 stripped) ──────
# Split by role, mirroring the include/realsense/{transmitter,receiver}
# tree: a shared wire lib, a Tx lib, an Rx lib, plus a guarded capture
# lib for the heavy librealsense2 dep. Depth path done; H.264 color next.

# RVL depth encoder (Module, pure). The H.264 color encoder (x264) is a
# separate target below. Publishers are folded into the encode-publisher
# threads (realsense_transmitter), so no standalone publisher libs.
add_library(realsense_rvl_encoder STATIC
    src/realsense/transmitter/rvl_encoder.cpp
)
target_include_directories(realsense_rvl_encoder PUBLIC include)

# Rx: the DDS subscribers (typed messages via kist_idl). The decoders now
# live inside their decode threads (realsense_receiver) — the color one
# carries the libav dep, so it's guarded there.
add_library(realsense_rx STATIC
    src/realsense/receiver/depth_subscriber.cpp
    src/realsense/receiver/color_subscriber.cpp
)
target_include_directories(realsense_rx PUBLIC include)
target_link_libraries(realsense_rx PUBLIC kist_idl unitree_sdk2)

# Heavy, role-specific deps found up front so the guards below all see
# them: x264 (Tx color), libav (Rx color), realsense2 (Tx capture).
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(X264 QUIET x264)
    pkg_check_modules(LIBAV QUIET libavcodec libavutil libswscale)
endif()
find_package(realsense2 QUIET)

# RealSense device reader (Thread) — Tx-only, librealsense2. Guarded.
if(realsense2_FOUND)
    add_library(realsense_capture STATIC src/realsense/transmitter/camera_capture.cpp)
    target_include_directories(realsense_capture PUBLIC include)
    target_link_libraries(realsense_capture PUBLIC realsense2::realsense2)
else()
    message(STATUS "librealsense2 not found — skipping realsense_capture (Tx capture)")
endif()

# H.264 color encoder — Tx-only, x264. Guarded: built only where x264 is.
if(X264_FOUND)
    add_library(realsense_h264_encoder STATIC
        src/realsense/transmitter/h264_encoder.cpp)
    target_include_directories(realsense_h264_encoder PUBLIC include PRIVATE ${X264_INCLUDE_DIRS})
    target_link_libraries(realsense_h264_encoder PUBLIC ${X264_LIBRARIES})
    target_link_directories(realsense_h264_encoder PUBLIC ${X264_LIBRARY_DIRS})
else()
    message(STATUS "x264 not found — skipping realsense_h264_encoder (Tx color)")
endif()

# ─── RealSense camera assemblies (embeddable libraries) ───────────────────────
# RealsenseTransmitter / RealsenseReceiver — own-no-thread assemblies that
# wire + start/stop the Tx/Rx threads. Consumers LINK these and use them
# in-process (Rx: read color()/depth()); the test_* executables below just
# run them. Config parsing stays in the runners, so these libs are yaml-free.

# Tx: capture (realsense2) + color encode (x264).
if(realsense2_FOUND AND X264_FOUND)
    add_library(realsense_transmitter STATIC
        src/system/realsense_transmitter.cpp
        src/realsense/transmitter/color_publisher.cpp
        src/realsense/transmitter/depth_publisher.cpp
    )
    target_include_directories(realsense_transmitter PUBLIC include)
    target_link_libraries(realsense_transmitter
        PUBLIC realsense_capture realsense_rvl_encoder realsense_h264_encoder
               kist_idl unitree_sdk2)
else()
    message(STATUS "realsense2 or x264 missing — skipping realsense_transmitter")
endif()

# Rx: color decode (libav); depth decode (RVL) is dep-free.
if(LIBAV_FOUND)
    add_library(realsense_receiver STATIC
        src/system/realsense_receiver.cpp
        src/realsense/receiver/color_decoder.cpp
        src/realsense/receiver/depth_decoder.cpp
    )
    target_include_directories(realsense_receiver PUBLIC include PRIVATE ${LIBAV_INCLUDE_DIRS})
    target_link_libraries(realsense_receiver PUBLIC realsense_rx ${LIBAV_LIBRARIES})
    target_link_directories(realsense_receiver PUBLIC ${LIBAV_LIBRARY_DIRS})
else()
    message(STATUS "libav missing — skipping realsense_receiver")
endif()

# ─── UWB assemblies (embeddable libraries) ────────────────────────────────────
# UwbTransmitter / UwbReceiver — own-no-thread assemblies (the mirror of the
# RealSense ones). Consumers LINK these and use them in-process (Rx: read
# fix()); the test_* runners below run them. yaml-free — config parsing lives
# in the runners.
add_library(uwb_transmitter STATIC src/system/uwb_transmitter.cpp)
target_include_directories(uwb_transmitter PUBLIC include)
target_link_libraries(uwb_transmitter PUBLIC uwb_io)

add_library(uwb_receiver STATIC src/system/uwb_receiver.cpp)
target_include_directories(uwb_receiver PUBLIC include)
target_link_libraries(uwb_receiver PUBLIC uwb_io)

# ─── tests ────────────────────────────────────────────────────────────────────
# Standalone runners for the RealSense assemblies — read config.yaml, start
# the assembly, and run (Tx publishes; Rx prints received fps). Run the two
# together (same machine via lo, or two machines) to check the transport.
# They also double as the reference for how to embed the assemblies.
if(TARGET realsense_transmitter)
    add_executable(test_realsense_transmitter test/test_realsense_transmitter.cpp)
    target_link_libraries(test_realsense_transmitter PRIVATE realsense_transmitter yaml-cpp)
endif()

if(TARGET realsense_receiver)
    add_executable(test_realsense_receiver test/test_realsense_receiver.cpp)
    target_link_libraries(test_realsense_receiver PRIVATE realsense_receiver yaml-cpp)
endif()

# Visual variant of the Rx runner — shows color | depth (window, or PNG when
# headless). Same embedding as test_realsense_receiver, plus OpenCV display.
find_package(OpenCV QUIET)
if(TARGET realsense_receiver AND OpenCV_FOUND)
    add_executable(test_realsense_receiver_viewer test/test_realsense_receiver_viewer.cpp)
    target_link_libraries(test_realsense_receiver_viewer
        PRIVATE realsense_receiver yaml-cpp ${OpenCV_LIBS})
    target_include_directories(test_realsense_receiver_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()

# UWB standalone runners — read config.yaml, run the assembly (Tx publishes
# from the DWM dongle; Rx prints received fixes). Also the embedding reference.
add_executable(test_uwb_transmitter test/test_uwb_transmitter.cpp)
target_link_libraries(test_uwb_transmitter PRIVATE uwb_transmitter yaml-cpp)

add_executable(test_uwb_receiver test/test_uwb_receiver.cpp)
target_link_libraries(test_uwb_receiver PRIVATE uwb_receiver yaml-cpp)

