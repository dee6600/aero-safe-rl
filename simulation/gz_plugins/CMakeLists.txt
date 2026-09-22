# M6: RotorDegradationSystem, the one Gazebo-side implementation of the
# rotor-degradation fault model (CLAUDE.md §1.4's one sanctioned "second
# implementation" exception -- the other half lives in M8b's Isaac-side
# Python rotor model, and must match this one, CLAUDE.md §1.6).
#
# Built entirely independently of PX4/ROS/colcon -- it only needs the gz-sim8
# / gz-transport13 / gz-msgs10 dev packages already on this machine
# (confirmed installed: libgz-sim8-dev, libgz-transport13-dev,
# libgz-msgs10-dev). scripts/sim_start.sh already points
# GZ_SIM_SYSTEM_PLUGIN_PATH at simulation/gz_plugins/build (from M1b), which
# is exactly where an out-of-source build places the .so -- no
# LIBRARY_OUTPUT_DIRECTORY override needed.
#
# Build:
#   cmake -S simulation/gz_plugins -B simulation/gz_plugins/build
#   cmake --build simulation/gz_plugins/build

cmake_minimum_required(VERSION 3.16)
project(aero_gz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(RotorDegradationSystem SHARED src/RotorDegradationSystem.cc)
target_link_libraries(RotorDegradationSystem
  PRIVATE
    gz-sim8::core
    gz-msgs10::core
    gz-transport13::core
)
