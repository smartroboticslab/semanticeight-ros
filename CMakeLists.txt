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
add_compile_options(-march=native)
option(WITH_OPENMP "Compile with OpenMP" ON)

set(field_type_macro SE_VOXEL_IMPLEMENTATION=OFusion)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
# Add local path for finding packages.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(supereight REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  message_generation
  message_filters
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  image_geometry
  tf
  cv_bridge
  eigen_conversions
)

add_message_files(FILES
  ImagePose.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
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
  rosbag
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  eigen_conversions
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
  ${CMAKE_THREAD_LIBS_INIT}
)



#add_dependencies(supereight_ros supereight_ros_generate_messages_cpp)
#install(TARGETS ${PROJECT_NAME}
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(DIRECTORY include/supereight_ros/
#        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/supereight_ros/
#        FILES_MATCHING PATTERN "*.h*"
#        )
#
#
##############
### Testing ##
##############
## command : catkin run_tests
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(supereight_ros-test
#        test/main.cpp
#        test/test_example.cpp
#        test/test_supereight_ros.cpp
#        )
#
#target_compile_definitions(supereight_ros-test PUBLIC ${field_type})
#target_link_libraries(supereight_ros-test
#        ${catkin_LIBRARIES} )
#add_dependencies(supereight_ros-test supereight_ros_generate_messages_cpp)
