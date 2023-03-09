# SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2019 Anna Dai
# SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10)

project(semanticeight_ros VERSION 0.1.0 LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# Compilation options
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
option(WITH_OPENMP "Compile with OpenMP" ON)
option(WITH_MASKRCNN "Compile with libmaskrcnn-trt" ON)
option(WITH_ASAN "Compile with the address sanitizer" OFF)
option(WITH_UBSAN "Compile with the undefined behavior sanitizer" OFF)

# Select the supereight voxel and sensor implementations to use.
set(VOXEL_IMPL MultiresOFusion MultiresTSDF)
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
  eigen_conversions
  geometry_msgs
  image_geometry
  mav_interface_msgs
  mav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf_conversions
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

if(WITH_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  link_libraries(-fsanitize=address)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    link_libraries(-static-libsan)
  else()
    link_libraries(-static-libasan)
  endif()
endif()

if(WITH_UBSAN)
  add_compile_options(-fsanitize=undefined)
  link_libraries(-fsanitize=undefined)
endif()

# Compile supereight
set(SE_VOXEL_IMPLS ${VOXEL_IMPL} CACHE STRING "The voxel implementations to compile")
set(SE_SENSOR_IMPLS ${SENSOR_IMPL} CACHE STRING "The camera implementations to compile")
option(SE_BUILD_TESTS "Build the supereight unit tests" OFF)
option(SE_BUILD_GLUT_GUI "Build the OpenGL-based GUI" OFF)
option(SE_BUILD_QT_GUI "Build the Qt-based GUI" OFF)
add_subdirectory(semanticeight)

# Compile libmaskrcnn-trt
if (WITH_MASKRCNN)
  find_package(CUDA)
  if (CUDA_FOUND)
      add_compile_options(-Wno-deprecated-declarations)
      add_subdirectory(libmaskrcnn-trt)
  else ()
      set(WITH_MASKRCNN OFF)
      message(WARNING "CUDA not found, not building with Mask R-CNN support")
  endif ()
endif ()

# Compile mesh-diff-tool
add_subdirectory(mesh-diff-tool "${CMAKE_CURRENT_SOURCE_DIR}/mesh-diff-tool/build")

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
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_geometry
  mav_interface_msgs
  mav_msgs
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf_conversions
  visualization_msgs
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
if (WITH_MASKRCNN)
  include_directories(libmaskrcnn-trt/include)
endif ()

# Compute the supereight library name from the voxel and sensor implementation
# names.
set(SUPEREIGHT_LIB "SE::DenseSLAM${SENSOR_IMPL}")


add_executable(semanticeight_ros
  "src/control_interface.cpp"
  "src/dataset.cpp"
  "src/stats.cpp"
  "src/utilities.cpp"
  "src/supereight_ros.cpp"
  "src/supereight_ros_config.cpp"
  "src/supereight_ros_visualization.cpp"
  "src/supereight_ros_node.cpp"
)


target_link_libraries(semanticeight_ros
  ${SUPEREIGHT_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  stdc++fs
)
if (WITH_MASKRCNN)
  target_link_libraries(semanticeight_ros maskrcnn-trt)
  target_compile_definitions(semanticeight_ros PUBLIC SE_WITH_MASKRCNN)
endif ()

# ------------------------------------------------------------------------------
# TEST
# ------------------------------------------------------------------------------
catkin_add_gtest(semanticeight_ros-unittests
  src/utilities.cpp
  test/utilities_unittest.cpp
)
target_include_directories(semanticeight_ros-unittests
  PUBLIC
    include
)
target_link_libraries(semanticeight_ros-unittests
  ${catkin_LIBRARIES}
  ${GTEST_BOTH_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

