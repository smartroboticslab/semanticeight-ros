# SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2019 Anna Dai
# SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10)

project(supereight_ros)
set(SE_ROS_MAJOR_VERSION 0)
set(SE_ROS_MINOR_VERSION 0)
set(SE_ROS_PATCH_VERSION 0)
set(SE_ROS_VERSION
  ${SE_ROS_MAJOR_VERSION}.${SE_ROS_MINOR_VERSION}.${SE_ROS_PATCH_VERSION})

# ------------------------------------------------------------------------------
# Compilation options
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
option(WITH_OPENMP "Compile with OpenMP" ON)

# Select the supereight voxel implementation to use.
set(field_type_macro SE_VOXEL_IMPLEMENTATION=OFusion)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
# Add local path for finding packages.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  eigen_conversions
  geometry_msgs
  image_geometry
  message_filters
  message_generation
  sensor_msgs
  std_msgs
  visualization_msgs
)

if (WITH_OPENMP)
  find_package(OpenMP)
  if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    message(STATUS "Compiling with OpenMP support")
  endif ()
endif ()

# Compile supereight
add_subdirectory(supereight)

# ------------------------------------------------------------------------------
# Catkin
# ------------------------------------------------------------------------------
# catkin specific configuration
# declare things to be passed to dependent projects
# supereight_ros_node and tests
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS
  roscpp
  message_runtime
  eigen_conversions
  geometry_msgs
  sensor_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# BUILD
# ------------------------------------------------------------------------------
## Specify additional locations of header files
## your package locations should be listed before other locations
## all targets in this CMakeList will have the directories added to their
## include path
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SUPEREIGHT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)



add_executable(supereight_ros
  "src/utilities.cpp"
  "src/supereight_ros_config.cpp"
  "src/supereight_ros.cpp"
  "src/supereight_ros_node.cpp"
)


target_compile_definitions(supereight_ros PUBLIC ${field_type_macro})

target_link_libraries(supereight_ros
  se-denseslam-ofusion
  lodepng
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# ------------------------------------------------------------------------------
# TEST
# ------------------------------------------------------------------------------
catkin_add_gtest(supereight_ros-unittests
  src/utilities.cpp
  test/utilities_unittest.cpp
)
target_include_directories(supereight_ros-unittests
  PUBLIC
    include
)
target_link_libraries(supereight_ros-unittests
  ${catkin_LIBRARIES}
  ${GTEST_BOTH_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

