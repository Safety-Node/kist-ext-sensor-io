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

# ─── camera_io ────────────────────────────────────────────────────────────────
# Camera Tx/Rx, ported from depth-cam-stream-codec with ROS2 stripped.
# Depth path done: RVL codec (pure) + wire (de)serialization + DDS
# Publisher/Receiver over the VoxelMapCompressed_ byte carrier (no custom
# IDL / idlc). RealSense capture and H.264 color are layered on next.
add_library(camera_io STATIC
    src/camera/rvl_depth_codec.cpp
    src/camera/rvl_depth_wire.cpp
    src/camera/depth_publisher.cpp
    src/camera/depth_receiver.cpp
)
target_include_directories(camera_io PUBLIC include)
target_link_libraries(camera_io PUBLIC unitree_sdk2)

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
target_link_libraries(test_rvl_codec PRIVATE camera_io)

add_executable(test_depth_wire test/test_depth_wire.cpp)
target_link_libraries(test_depth_wire PRIVATE camera_io)

add_executable(test_depth_loopback test/test_depth_loopback.cpp)
target_link_libraries(test_depth_loopback PRIVATE camera_io)

# ─── camera_capture ───────────────────────────────────────────────────────────
# RealSense device reader (Thread) — Tx-only, so its heavy librealsense2
# dependency stays off camera_io. Built only when librealsense2 is found.
find_package(realsense2 QUIET)
if(realsense2_FOUND)
    add_library(camera_capture STATIC src/camera/camera_capture.cpp)
    target_include_directories(camera_capture PUBLIC include)
    target_link_libraries(camera_capture PUBLIC realsense2::realsense2)
else()
    message(STATUS "librealsense2 not found — skipping camera_capture (Tx capture)")
endif()

# Visual probes (OpenCV) — developer eyeball checks, not linked by the lib.
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_executable(test_depth_view test/test_depth_view.cpp)
    target_link_libraries(test_depth_view PRIVATE camera_io ${OpenCV_LIBS})
    target_include_directories(test_depth_view PRIVATE ${OpenCV_INCLUDE_DIRS})

    if(realsense2_FOUND)
        add_executable(test_camera_view test/test_camera_view.cpp)
        target_link_libraries(test_camera_view PRIVATE camera_io camera_capture ${OpenCV_LIBS})
        target_include_directories(test_camera_view PRIVATE ${OpenCV_INCLUDE_DIRS})
    endif()
else()
    message(STATUS "OpenCV not found — skipping visual probes")
endif()
