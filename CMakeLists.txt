# SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2019 Anna Dai
# SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10)

project(supereight_ros VERSION 0.1.0 LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# Compilation options
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
option(WITH_OPENMP "Compile with OpenMP" ON)

# Select the supereight voxel and sensor implementations to use.
set(VOXEL_IMPL MultiresOFusion)
set(SENSOR_IMPL PinholeCamera)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
# Add local path for finding packages.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  eigen_conversions
  geometry_msgs
  image_geometry
  message_filters
  message_generation
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
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
set(SE_VOXEL_IMPLS ${VOXEL_IMPL} CACHE STRING "The voxel implementations to compile")
set(SE_SENSOR_IMPLS ${SENSOR_IMPL} CACHE STRING "The camera implementations to compile")
option(SE_BUILD_TESTS "Build the supereight unit tests" OFF)
option(SE_BUILD_GLUT_GUI "Build the OpenGL-based GUI" OFF)
option(SE_BUILD_QT_GUI "Build the Qt-based GUI" OFF)
add_subdirectory(semanticeight)

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
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

# Compute the supereight library name from the voxel and sensor implementation
# names.
set(SUPEREIGHT_LIB "SE::DenseSLAM${VOXEL_IMPL}${SENSOR_IMPL}")


add_executable(supereight_ros
  "src/utilities.cpp"
  "src/supereight_ros_config.cpp"
  "src/supereight_ros.cpp"
  "src/supereight_ros_node.cpp"
)


target_link_libraries(supereight_ros
  ${SUPEREIGHT_LIB}
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

