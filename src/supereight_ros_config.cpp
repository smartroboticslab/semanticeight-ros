// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros_config.hpp"

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "se/sensor_implementation.hpp"
#include "se/voxel_implementations.hpp"



// Default supereight node configuration.
constexpr bool default_enable_tracking = true;
constexpr bool default_enable_rendering = true;
constexpr bool default_enable_rgb = false;
const Eigen::Vector2i default_input_res (640, 480);
const std::string default_pose_topic_type ("geometry_msgs::PoseStamped");
constexpr int default_pose_buffer_size = 600;
constexpr int default_depth_buffer_size = 60;
constexpr int default_rgb_buffer_size = 60;
constexpr double default_max_timestamp_diff = 0.001;
constexpr bool default_center_at_first_position = true;
constexpr int default_visualization_rate = 4;
constexpr float default_visualization_max_z = INFINITY;



namespace se {
  SupereightNodeConfig read_supereight_node_config(const ros::NodeHandle& nh) {
    SupereightNodeConfig config;

    nh.param<bool>("supereight_ros/enable_tracking",
        config.enable_tracking, default_enable_tracking);

    nh.param<bool>("supereight_ros/enable_rendering",
        config.enable_rendering, default_enable_rendering);

    nh.param<bool>("supereight_ros/enable_rgb",
        config.enable_rgb, default_enable_rgb);

    std::vector<int> input_res_vector;
    if (nh.getParam("supereight_ros/input_res", input_res_vector)) {
      for (size_t i = 0; i < input_res_vector.size(); ++i) {
        config.input_res[i] = input_res_vector[i];
      }
    } else {
      config.input_res = default_input_res;
    }

    nh.param<std::string>("supereight_ros/pose_topic_type",
        config.pose_topic_type, default_pose_topic_type);

    nh.param<int>("supereight_ros/pose_buffer_size",
        config.pose_buffer_size, default_pose_buffer_size);

    nh.param<int>("supereight_ros/depth_buffer_size",
        config.depth_buffer_size, default_depth_buffer_size);

    nh.param<int>("supereight_ros/rgb_buffer_size",
        config.rgb_buffer_size, default_rgb_buffer_size);

    nh.param<double>("supereight_ros/max_timestamp_diff",
        config.max_timestamp_diff, default_max_timestamp_diff);

    nh.param<bool>("supereight_ros/center_at_first_position",
        config.center_at_first_position, default_center_at_first_position);

    nh.param<int>("supereight_ros/visualization_rate",
        config.visualization_rate, default_visualization_rate);

    nh.param<float>("supereight_ros/visualization_max_z",
        config.visualization_max_z, default_visualization_max_z);

    return config;
  }



  void print_supereight_node_config(const SupereightNodeConfig& config) {
    ROS_INFO("Supereight Node parameters:");
    ROS_INFO("  enable_tracking:          %d", config.enable_tracking);
    ROS_INFO("  enable_rendering:         %d", config.enable_rendering);
    ROS_INFO("  enable_rgb:               %d", config.enable_rgb);
    ROS_INFO("  input_res:                %d %d", config.input_res.x(), config.input_res.y());
    ROS_INFO("  pose_topic_type:          %s", config.pose_topic_type.c_str());
    ROS_INFO("  pose_buffer_size:         %d", config.pose_buffer_size);
    ROS_INFO("  depth_buffer_size:        %d", config.depth_buffer_size);
    ROS_INFO("  rgb_buffer_size:          %d", config.rgb_buffer_size);
    ROS_INFO("  max_timestamp_diff:       %f", config.max_timestamp_diff);
    ROS_INFO("  center_at_first_position: %d", config.center_at_first_position);
    ROS_INFO("  visualization_rate:       %d", config.visualization_rate);
    ROS_INFO("  visualization_max_z:      %f", config.visualization_max_z);
  }



  Configuration read_supereight_config(const ros::NodeHandle& nh) {
    Configuration config;

    // General
    nh.getParam("supereight/general/sequence_name", config.sequence_name);
    nh.getParam("supereight/general/sequence_type", config.sequence_type);
    bool enable_tracking = false;
    nh.getParam("supereight_ros/enable_tracking", enable_tracking);
    config.enable_ground_truth = !enable_tracking;
    nh.getParam("supereight_ros/enable_rendering", config.enable_render);
    nh.getParam("supereight/general/output_render_path", config.output_render_file);
    nh.getParam("supereight/general/enable_meshing", config.enable_meshing);
    nh.getParam("supereight/general/output_mesh_path", config.output_mesh_file);
    nh.getParam("supereight/general/tracking_rate", config.tracking_rate);
    nh.getParam("supereight/general/integration_rate", config.integration_rate);
    nh.getParam("supereight/general/rendering_rate", config.rendering_rate);
    nh.getParam("supereight/general/meshing_rate", config.meshing_rate);
    nh.getParam("supereight/general/fps", config.fps);
    nh.getParam("supereight/general/drop_frames", config.drop_frames);
    nh.getParam("supereight/general/max_frame", config.max_frame);
    nh.getParam("supereight/general/icp_threshold", config.icp_threshold);
    nh.getParam("supereight/general/bilateral_filter", config.bilateral_filter);
    nh.getParam("supereight/general/pyramid", config.pyramid);

    // Map
    int map_size;
    nh.getParam("supereight/map/size", map_size);
    config.map_size = Eigen::Vector3i::Constant(map_size);

    float map_dim;
    nh.getParam("supereight/map/dim", map_dim);
    config.map_dim = Eigen::Vector3f::Constant(map_dim);

    std::vector<float> t_MW_factor_vector;
    if (nh.getParam("supereight/map/t_MW_factor", t_MW_factor_vector)) {
      for (size_t i = 0; i < t_MW_factor_vector.size(); ++i) {
        config.t_MW_factor[i] = t_MW_factor_vector[i];
      }
    }

    // Sensor
    config.sensor_type = SensorImpl::type();

    std::vector<float> sensor_intrinsics_vector;
    if (!nh.getParam("supereight/sensor/intrinsics", sensor_intrinsics_vector)
        || sensor_intrinsics_vector.empty()) {
      ros::shutdown();
    }
    for (size_t i = 0; i < sensor_intrinsics_vector.size(); i++) {
      config.sensor_intrinsics[i] = sensor_intrinsics_vector[i];
    }
    config.left_hand_frame = config.sensor_intrinsics[1] < 0;

    nh.getParam("supereight/sensor/downsampling_factor", config.sensor_downsampling_factor);

    std::vector<float> T_BC_vector;
    nh.getParam("supereight/sensor/T_BC", T_BC_vector);
    if (nh.getParam("supereight/sensor/T_BC", T_BC_vector)) {
      for (size_t i = 0; i < std::sqrt(T_BC_vector.size()); ++i) {
        for (size_t j = 0; j < std::sqrt(T_BC_vector.size()); ++j) {
          config.T_BC(i, j) = T_BC_vector[i * 4 + j];
        }
      }
    }

    std::vector<float> init_T_WB_vector;
    nh.getParam("supereight/sensor/init_T_WB", init_T_WB_vector);
    if (nh.getParam("supereight/sensor/init_T_WB", init_T_WB_vector)) {
      for (size_t i = 0; i < std::sqrt(init_T_WB_vector.size()); ++i) {
        for (size_t j = 0; j < std::sqrt(init_T_WB_vector.size()); ++j) {
          config.init_T_WB(i, j) = init_T_WB_vector[i * 4 + j];
        }
      }
    }

    nh.getParam("supereight/sensor/near_plane", config.near_plane);

    nh.getParam("supereight/sensor/far_plane", config.far_plane);

    // Voxel Impl
    config.voxel_impl_type = VoxelImpl::type();
    const float voxel_dim = config.map_dim.x() / config.map_size.x();
    VoxelImpl::configure(voxel_dim);
    // TODO load the voxel implementation parameters from the YAML file.
    // Couldn't find a way to get the filename of the YAML file loaded from ROS.

    return config;
  }

} // namespace se

