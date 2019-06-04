cmake_minimum_required(VERSION 3.10)
project(supereight_ros)

add_compile_options(-std=c++14 -march=native)

add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)
add_definitions(-DWITH_RENDERING)

message(WARNING "CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})
message(WARNING "CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})
# ------------------------------------------------------------------------------
# Prepare dependencies
# ------------------------------------------------------------------------------

option(WITH_OPENMP "Compile with OpenMP" ON)
find_package(Eigen3 REQUIRED)
find_package(supereight REQUIRED)
find_package(GLUT)
find_package(OpenGL)

## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        message_filters
        rosbag
        roscpp
        roslib
        std_msgs
        message_generation
        visualization_msgs
        geometry_msgs
        image_geometry
        tf
        eigen_conversions
        )

add_message_files(
        FILES
        ImagePose.msg
        )

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
)

if (WITH_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message(STATUS "Compiling with OpenMP support")
    endif ()
endif ()
#
if (WITH_OPENMP AND OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
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
        geometry_msgs
        std_msgs
        message_runtime
        cv_bridge
        tf
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
        src/supereight_ros_node.cpp
        )

#############################################
# Supereight Map Options
#############################################
## Used for supereight package
# Options for SE_FIELD_TYPE: OFusion, Sdf
#if (MAP_OM)
#    message("Compile supereight with OFusion")
#    set(CMAKE_CXX_FLAGS
#            "${CMAKE_CXX_FLAGS} -DMAP_OM")
#    set(field_type SE_FIELD_TYPE=OFusion)
#
#elseif (MAP_SDF)
#    message("Compile supereight with SDF")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAP_SDF")
#    set(field_type SE_FIELD_TYPE=SDF)
#
#elseif (NOT MAP_OM AND NOT MAP_SDF)
#    message("Compile supereight with OFusion")
#    set(CMAKE_CXX_FLAGS
#            "${CMAKE_CXX_FLAGS} -DMAP_OM")
    set(field_type SE_FIELD_TYPE=OFusion)
#endif (MAP_OM)


target_compile_definitions(supereight_ros PUBLIC ${field_type})

target_link_libraries(supereight_ros
        se-denseslam-ofusion
        se-denseslam-sdf
        lodepng
        ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES} GL)

add_dependencies(supereight_ros supereight_ros_generate_messages_cpp)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/supereight_ros/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/supereight_ros/
        FILES_MATCHING PATTERN "*.h*"
        )


#############
## Testing ##
#############
# command : catkin run_tests
# Add gtest based cpp test target and link libraries
catkin_add_gtest(supereight_ros-test
        test/main.cpp
        test/test_example.cpp
        test/test_supereight_ros.cpp
        )

target_compile_definitions(supereight_ros-test PUBLIC ${field_type})
target_link_libraries(supereight_ros-test
        ${catkin_LIBRARIES} )
add_dependencies(supereight_ros-test supereight_ros_generate_messages_cpp)
