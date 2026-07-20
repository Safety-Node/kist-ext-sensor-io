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
# UWB Tx/Rx pair: DwmSerial (serial -> samples) + UwbTransmitter
# (samples -> DDS rt/kist/uwb/pose) + UwbReceiver (DDS -> uwb_buf,
# set_on_position hook for wrapper wiring).
add_library(uwb_io STATIC
    src/uwb/dwm_serial.cpp
    src/uwb/uwb_transmitter.cpp
    src/uwb/uwb_receiver.cpp
)
target_include_directories(uwb_io PUBLIC include)
target_link_libraries(uwb_io PUBLIC unitree_sdk2 yaml-cpp)

# ─── apps ─────────────────────────────────────────────────────────────────────
# Onboard daemon: runs on the machine the DWM dongle is plugged into.
add_executable(uwb_transmitter apps/uwb_transmitter_main.cpp)
target_link_libraries(uwb_transmitter PRIVATE uwb_io yaml-cpp)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_dwm_parse test/test_dwm_parse.cpp)
target_link_libraries(test_dwm_parse PRIVATE uwb_io)

add_executable(test_uwb_loopback test/test_uwb_loopback.cpp)
target_link_libraries(test_uwb_loopback PRIVATE uwb_io)

add_executable(test_uwb_receiver test/test_uwb_receiver.cpp)
target_link_libraries(test_uwb_receiver PRIVATE uwb_io)
