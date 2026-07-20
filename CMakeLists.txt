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

# ─── uwb_io ───────────────────────────────────────────────────────────────────
# UWB Tx/Rx, one class per thread: UwbSerialReader (serial -> samples)
# + UwbPublisher (samples -> DDS) + UwbTransmitter (their assembly) +
# UwbReceiver (DDS -> uwb_buf, set_on_position hook for wrapper wiring).
add_library(uwb_io STATIC
    src/uwb/uwb_serial_reader.cpp
    src/uwb/uwb_transmitter.cpp
    src/uwb/uwb_publisher.cpp
    src/uwb/uwb_receiver.cpp
)
target_include_directories(uwb_io PUBLIC include)
target_link_libraries(uwb_io PUBLIC unitree_sdk2 yaml-cpp)

# ─── realsense camera (ported from depth-cam-stream-codec, ROS2 stripped) ──────
# Split by role, mirroring the include/realsense/{transmitter,receiver}
# tree: a shared wire lib, a Tx lib, an Rx lib, plus a guarded capture
# lib for the heavy librealsense2 dep. Depth path done; H.264 color next.

# shared: byte (de)serialization of compressed frames (no deps).
add_library(realsense_wire STATIC
    src/realsense/rvl_depth_wire.cpp
    src/realsense/h264_color_wire.cpp
)
target_include_directories(realsense_wire PUBLIC include)

# Tx: light encoders + DDS publishers (VoxelMapCompressed_ carrier). The
# heavy H.264 encoder (x264) is a separate target below.
add_library(realsense_tx STATIC
    src/realsense/transmitter/rvl_depth_encoder.cpp
    src/realsense/transmitter/depth_publisher.cpp
    src/realsense/transmitter/color_publisher.cpp
)
target_include_directories(realsense_tx PUBLIC include)
target_link_libraries(realsense_tx PUBLIC realsense_wire unitree_sdk2)

# Rx: light decoders + DDS receivers. The heavy H.264 decoder (libav) is
# a separate target below.
add_library(realsense_rx STATIC
    src/realsense/receiver/rvl_depth_decoder.cpp
    src/realsense/receiver/depth_receiver.cpp
    src/realsense/receiver/color_receiver.cpp
)
target_include_directories(realsense_rx PUBLIC include)
target_link_libraries(realsense_rx PUBLIC realsense_wire unitree_sdk2)

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
        src/realsense/transmitter/h264_color_encoder.cpp)
    target_include_directories(realsense_h264_encoder PUBLIC include PRIVATE ${X264_INCLUDE_DIRS})
    target_link_libraries(realsense_h264_encoder PUBLIC ${X264_LIBRARIES})
    target_link_directories(realsense_h264_encoder PUBLIC ${X264_LIBRARY_DIRS})
else()
    message(STATUS "x264 not found — skipping realsense_h264_encoder (Tx color)")
endif()

# H.264 color decoder — Rx-only, libavcodec/libswscale. Guarded.
if(LIBAV_FOUND)
    add_library(realsense_h264_decoder STATIC
        src/realsense/receiver/h264_color_decoder.cpp)
    target_include_directories(realsense_h264_decoder PUBLIC include PRIVATE ${LIBAV_INCLUDE_DIRS})
    target_link_libraries(realsense_h264_decoder PUBLIC ${LIBAV_LIBRARIES})
    target_link_directories(realsense_h264_decoder PUBLIC ${LIBAV_LIBRARY_DIRS})
else()
    message(STATUS "libav not found — skipping realsense_h264_decoder (Rx color)")
endif()

# ─── camera assemblies (threaded Tx/Rx, one class per thread) ──────────────────
# CameraTransmitter needs capture (realsense2) + color encode (x264);
# built only where both are available (the robot side).
if(realsense2_FOUND AND X264_FOUND)
    add_library(realsense_transmitter STATIC
        src/realsense/transmitter/color_encode_publisher.cpp
        src/realsense/transmitter/depth_encode_publisher.cpp
        src/realsense/transmitter/camera_transmitter.cpp
    )
    target_include_directories(realsense_transmitter PUBLIC include)
    target_link_libraries(realsense_transmitter
        PUBLIC realsense_capture realsense_tx realsense_h264_encoder)
else()
    message(STATUS "realsense2 or x264 missing — skipping realsense_transmitter")
endif()

# CameraReceiver needs color decode (libav); depth decode is dep-free.
if(LIBAV_FOUND)
    add_library(realsense_receiver STATIC
        src/realsense/receiver/color_decode_thread.cpp
        src/realsense/receiver/depth_decode_thread.cpp
        src/realsense/receiver/camera_receiver.cpp
    )
    target_include_directories(realsense_receiver PUBLIC include)
    target_link_libraries(realsense_receiver
        PUBLIC realsense_rx realsense_h264_decoder)
else()
    message(STATUS "libav missing — skipping realsense_receiver")
endif()

# ─── system facades ───────────────────────────────────────────────────────────
# Tx: device-machine assembly (starts every enabled sensor's serial→DDS
# chain). Rx: consumer-machine assembly (starts every enabled sensor's
# Receiver; data access stays per sensor). One block per sensor inside.
add_library(ext_sensor_io_tx STATIC
    src/system/ext_sensor_io_tx.cpp
)
target_include_directories(ext_sensor_io_tx PUBLIC include)
target_link_libraries(ext_sensor_io_tx PUBLIC uwb_io)

add_library(ext_sensor_io_rx STATIC
    src/system/ext_sensor_io_rx.cpp
)
target_include_directories(ext_sensor_io_rx PUBLIC include)
target_link_libraries(ext_sensor_io_rx PUBLIC uwb_io)

# ─── daemon ───────────────────────────────────────────────────────────────────
# The Tx daemon: runs on the device machine (dongle/camera/mic side).
add_executable(ext_sensor_io_tx_daemon src/main.cpp)
set_target_properties(ext_sensor_io_tx_daemon PROPERTIES OUTPUT_NAME ext_sensor_io_tx)
target_link_libraries(ext_sensor_io_tx_daemon PRIVATE ext_sensor_io_tx)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_dwm_parse test/test_dwm_parse.cpp)
target_link_libraries(test_dwm_parse PRIVATE uwb_io)

add_executable(test_uwb_loopback test/test_uwb_loopback.cpp)
target_link_libraries(test_uwb_loopback PRIVATE uwb_io)

add_executable(test_uwb_receiver test/test_uwb_receiver.cpp)
target_link_libraries(test_uwb_receiver PRIVATE ext_sensor_io_rx)

add_executable(test_rvl_codec test/test_rvl_codec.cpp)
target_link_libraries(test_rvl_codec PRIVATE realsense_tx realsense_rx)

add_executable(test_depth_wire test/test_depth_wire.cpp)
target_link_libraries(test_depth_wire PRIVATE realsense_wire)

add_executable(test_depth_loopback test/test_depth_loopback.cpp)
target_link_libraries(test_depth_loopback PRIVATE realsense_tx realsense_rx)

# H.264 codec + color DDS loopback need both x264 (encode) and libav (decode).
if(X264_FOUND AND LIBAV_FOUND)
    add_executable(test_h264_codec test/test_h264_codec.cpp)
    target_link_libraries(test_h264_codec
        PRIVATE realsense_h264_encoder realsense_h264_decoder realsense_wire)

    add_executable(test_color_loopback test/test_color_loopback.cpp)
    target_link_libraries(test_color_loopback
        PRIVATE realsense_h264_encoder realsense_h264_decoder realsense_tx realsense_rx)
endif()

# Visual probes (OpenCV) — developer eyeball checks, not linked by the lib.
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_executable(test_depth_view test/test_depth_view.cpp)
    target_link_libraries(test_depth_view PRIVATE realsense_tx realsense_rx ${OpenCV_LIBS})
    target_include_directories(test_depth_view PRIVATE ${OpenCV_INCLUDE_DIRS})

    if(realsense2_FOUND)
        add_executable(test_camera_view test/test_camera_view.cpp)
        target_link_libraries(test_camera_view PRIVATE realsense_tx realsense_rx realsense_capture ${OpenCV_LIBS})
        target_include_directories(test_camera_view PRIVATE ${OpenCV_INCLUDE_DIRS})

        if(X264_FOUND AND LIBAV_FOUND)
            add_executable(test_color_view test/test_color_view.cpp)
            target_link_libraries(test_color_view
                PRIVATE realsense_capture realsense_h264_encoder realsense_h264_decoder ${OpenCV_LIBS})
            target_include_directories(test_color_view PRIVATE ${OpenCV_INCLUDE_DIRS})

            # Both pipelines at once: color (H.264) + depth (RVL) over DDS.
            add_executable(test_realsense_view test/test_realsense_view.cpp)
            target_link_libraries(test_realsense_view PRIVATE
                realsense_capture realsense_tx realsense_rx
                realsense_h264_encoder realsense_h264_decoder ${OpenCV_LIBS})
            target_include_directories(test_realsense_view PRIVATE ${OpenCV_INCLUDE_DIRS})
        endif()
    endif()

    # Rx probe: live depth viewer for the two-machine split test (OpenCV,
    # no RealSense needed — it only receives).
    add_executable(test_camera_rx test/test_camera_rx.cpp)
    target_link_libraries(test_camera_rx PRIVATE realsense_rx yaml-cpp ${OpenCV_LIBS})
    target_include_directories(test_camera_rx PRIVATE ${OpenCV_INCLUDE_DIRS})

    # Two-machine Rx: full CameraReceiver (color+depth) live viewer.
    if(TARGET realsense_receiver)
        add_executable(test_realsense_rx test/test_realsense_rx.cpp)
        target_link_libraries(test_realsense_rx PRIVATE realsense_receiver yaml-cpp ${OpenCV_LIBS})
        target_include_directories(test_realsense_rx PRIVATE ${OpenCV_INCLUDE_DIRS})
    endif()
else()
    message(STATUS "OpenCV not found — skipping visual probes")
endif()

# Tx probe: depth-only capture -> encode -> publish (needs the RealSense).
if(realsense2_FOUND)
    add_executable(test_camera_tx test/test_camera_tx.cpp)
    target_link_libraries(test_camera_tx PRIVATE realsense_tx realsense_capture yaml-cpp)
endif()

# Two-machine Tx: full CameraTransmitter (color+depth).
if(TARGET realsense_transmitter)
    add_executable(test_realsense_tx test/test_realsense_tx.cpp)
    target_link_libraries(test_realsense_tx PRIVATE realsense_transmitter yaml-cpp)
endif()
