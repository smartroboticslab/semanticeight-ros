// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros_config.hpp"

#include <string>
#include <vector>



// Default supereight configuration.
constexpr int default_iteration_count = 3;
constexpr int default_iterations[default_iteration_count] = {10, 5, 4};
constexpr int default_compute_size_ratio = 2;
constexpr int default_tracking_rate = 1;
constexpr int default_integration_rate = 3;
constexpr int default_rendering_rate = 4;
const Eigen::Vector3i default_volume_resolution(256, 256, 256);
const Eigen::Vector3f default_volume_size(6.f, 6.f, 6.f);
const Eigen::Vector3f default_initial_pos_factor(0.5f, 0.5f, 0.0f);
const std::string default_dump_volume_file = "";
const Eigen::Matrix4f default_T_BC = Eigen::Matrix4f::Identity();
constexpr float default_mu = 0.1f;
constexpr int default_fps = 0;
constexpr bool default_blocking_read = false;
constexpr float default_icp_threshold = 1e-6;
constexpr bool default_no_gui = false;
constexpr bool default_render_volume_fullsize = false;
constexpr bool default_bilateral_filter = false;



// Default supereight node configuration.
constexpr bool default_enable_tracking = true;
constexpr bool default_enable_rendering = true;
constexpr bool default_process_rgb = false;
const Eigen::Vector2i default_input_res (640, 480);
const std::string default_pose_topic_type ("geometry_msgs::PoseStamped");
constexpr int default_pose_buffer_size = 600;
constexpr int default_depth_buffer_size = 60;
constexpr int default_rgb_buffer_size = 60;
constexpr double default_max_timestamp_diff = 0.001;



namespace se {
  SupereightNodeConfig read_supereight_node_config(const ros::NodeHandle& nh) {
    SupereightNodeConfig config;

    nh.param<bool>("enable_tracking",
        config.enable_tracking,
        default_enable_tracking);

    nh.param<bool>("enable_rendering",
        config.enable_rendering,
        default_enable_rendering);

    nh.param<bool>("enable_rgb",
        config.enable_rgb,
        default_process_rgb);

    std::vector<int> input_res_vector;
    if (nh.getParam("input_res", input_res_vector)) {
      for (size_t i = 0; i < input_res_vector.size(); ++i) {
        config.input_res[i] = input_res_vector[i];
      }
    } else {
      config.input_res = default_input_res;
    }

    nh.param<std::string>("pose_topic_type",
        config.pose_topic_type,
        default_pose_topic_type);

    nh.param<int>("pose_buffer_size",
        config.pose_buffer_size,
        default_pose_buffer_size);

    nh.param<int>("depth_buffer_size",
        config.depth_buffer_size,
        default_depth_buffer_size);

    nh.param<int>("rgb_buffer_size",
        config.rgb_buffer_size,
        default_rgb_buffer_size);

    nh.param<double>("max_timestamp_diff",
        config.max_timestamp_diff,
        default_max_timestamp_diff);

    return config;
  }



  void print_supereight_node_config(const SupereightNodeConfig& config) {
    ROS_INFO("Supereight Node parameters:");
    ROS_INFO("  enable_tracking:        %d",
        config.enable_tracking);
    ROS_INFO("  enable_rendering:       %d",
        config.enable_rendering);
    ROS_INFO("  enable_rgb:             %d",
        config.enable_rgb);
    ROS_INFO("  input_res:              %d %d",
        config.input_res.x(), config.input_res.y());
    ROS_INFO("  pose_topic_type:        %s",
        config.pose_topic_type.c_str());
    ROS_INFO("  pose_buffer_size:       %d",
        config.pose_buffer_size);
    ROS_INFO("  depth_buffer_size:      %d",
        config.depth_buffer_size);
    ROS_INFO("  rgb_buffer_size:        %d",
        config.rgb_buffer_size);
    ROS_INFO("  max_timestamp_diff:     %f",
        config.max_timestamp_diff);
  }



  Configuration read_supereight_config(const ros::NodeHandle& nh) {
    Configuration config;

    // Initialize unused parameters.
    config.input_file = "";
    config.log_file = "";
    config.groundtruth_file = "";

    // Read the other parameters.
    nh.param<int>("compute_size_ratio",
        config.compute_size_ratio,
        default_compute_size_ratio);

    nh.param<int>("tracking_rate",
        config.tracking_rate,
        default_tracking_rate);

    nh.param<int>("integration_rate",
        config.integration_rate,
        default_integration_rate);

    nh.param<int>("rendering_rate",
        config.rendering_rate,
        default_rendering_rate);

    std::vector<int> volume_resolution_vector;
    if (nh.getParam("volume_resolution", volume_resolution_vector)) {
      for (size_t i = 0; i < volume_resolution_vector.size(); ++i) {
        config.volume_resolution[i] = volume_resolution_vector[i];
      }
    } else {
      config.volume_resolution = default_volume_resolution;
    }

    std::vector<float> volume_size_vector;
    if (nh.getParam("volume_size", volume_size_vector)) {
      for (size_t i = 0; i < volume_size_vector.size(); ++i) {
        config.volume_size[i] = volume_size_vector[i];
      }
    } else {
      config.volume_size = default_volume_size;
    }

    std::vector<float> initial_pos_factor_vector;
    if (nh.getParam("initial_pos_factor", initial_pos_factor_vector)) {
      for (size_t i = 0; i < initial_pos_factor_vector.size(); ++i) {
        config.initial_pos_factor[i] = initial_pos_factor_vector[i];
      }
    } else {
      config.initial_pos_factor = default_initial_pos_factor;
    }

    std::vector<int> pyramid;
    if (!nh.getParam("pyramid", pyramid)) {
      config.pyramid.clear();
      for (int i = 0; i < default_iteration_count; ++i) {
        config.pyramid.push_back(default_iterations[i]);
      }
    }

    nh.param<std::string>("dump_volume_file",
        config.dump_volume_file,
        default_dump_volume_file);

    std::vector<float> T_BC_vector;
    nh.getParam("T_BC", T_BC_vector);
    if (nh.getParam("T_BC", T_BC_vector)) {
      for (size_t i = 0; i < std::sqrt(T_BC_vector.size()); ++i) {
        for (size_t j = 0; j < std::sqrt(T_BC_vector.size()); ++j) {
          config.T_BC(i, j) = T_BC_vector[i * 4 + j];
        }
      }
    } else {
      config.T_BC = default_T_BC;
    }

    std::vector<float> camera_vector;
    if (!nh.getParam("camera", camera_vector)) {
      ros::shutdown();
    }
    for (size_t i = 0; i < camera_vector.size(); i++) {
      config.camera[i] = camera_vector[i];
    }
    config.camera_overrided = true;

    nh.param<float>("mu",
        config.mu,
        default_mu);

    nh.param<int>("fps",
        config.fps,
        default_fps);

    nh.param<bool>("blocking_read",
        config.blocking_read,
        default_blocking_read);

    nh.param<float>("icp_threshold",
        config.icp_threshold,
        default_icp_threshold);

    nh.param<bool>("no_gui",
        config.no_gui,
        default_no_gui);

    nh.param<bool>("render_volume_fullsize",
        config.render_volume_fullsize,
        default_render_volume_fullsize);

    nh.param<bool>("bilateral_filter",
        config.bilateral_filter,
        default_bilateral_filter);

    return config;
  }



  void print_supereight_config(const Configuration& config) {
    ROS_INFO("Supereight parameters:");
    ROS_INFO("  compute_size_ratio:     %d",
        config.compute_size_ratio);
    ROS_INFO("  tracking_rate:          %d",
        config.tracking_rate);
    ROS_INFO("  integration_rate:       %d",
        config.integration_rate);
    ROS_INFO("  rendering_rate:         %d",
        config.rendering_rate);
    ROS_INFO("  volume_resolution:      %d %d %d",
        config.volume_resolution.x(),
        config.volume_resolution.y(),
        config.volume_resolution.z());
    ROS_INFO("  volume_size:            %f %f %f",
        config.volume_size.x(),
        config.volume_size.y(),
        config.volume_size.z());
    ROS_INFO("  initial_pos_factor:     %f %f %f",
        config.initial_pos_factor.x(),
        config.initial_pos_factor.y(),
        config.initial_pos_factor.z());
    ROS_INFO("  pyramid:                %d %d %d",
        config.pyramid[0],
        config.pyramid[1],
        config.pyramid[2]);
    ROS_INFO("  dump_volume_file:       \"%s\"",
        config.dump_volume_file.c_str());
    ROS_INFO("  T_BC:");
    for (size_t i = 0; i < 4; ++i) {
      ROS_INFO("                          %f %f %f %f",
          config.T_BC(4 * i + 0),
          config.T_BC(4 * i + 1),
          config.T_BC(4 * i + 2),
          config.T_BC(4 * i + 3));
    }
    ROS_INFO("  camera:                 %f %f %f %f",
        config.camera.x(),
        config.camera.y(),
        config.camera.z(),
        config.camera.w());
    ROS_INFO("  camera_overrided:       %d",
        config.camera_overrided);
    ROS_INFO("  mu:                     %f",
        config.mu);
    ROS_INFO("  fps:                    %d",
        config.fps);
    ROS_INFO("  blocking_read:          %d",
        config.blocking_read);
    ROS_INFO("  icp_threshold:          %f",
        config.icp_threshold);
    ROS_INFO("  no_gui:                 %d",
        config.no_gui);
    ROS_INFO("  render_volume_fullsize: %d",
        config.render_volume_fullsize);
    ROS_INFO("  bilateral_filter:       %d",
        config.bilateral_filter);
  }

} // namespace se

