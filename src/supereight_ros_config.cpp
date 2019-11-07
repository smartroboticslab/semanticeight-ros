//
// Created by anna on 16/05/19.
//

#include "supereight_ros/supereight_ros_config.hpp"

#include <string>
#include <vector>

#include <Eigen/Dense>



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
const std::string default_input_file = "";
const std::string default_log_file = "";
const std::string default_groundtruth_file = "";
const Eigen::Matrix4f default_gt_transform = Eigen::Matrix4f::Identity();
constexpr float default_mu = 0.1f;
constexpr int default_fps = 0;
constexpr bool default_blocking_read = false;
constexpr float default_icp_threshold = 1e-6;
constexpr bool default_no_gui = false;
constexpr bool default_render_volume_fullsize = false;
constexpr bool default_bilateral_filter = false;
constexpr int default_coloured_voxels = false;
constexpr int default_multi_resolution = false;
constexpr bool default_bayesian = false;



namespace se {
  Configuration read_supereight_config(const ros::NodeHandle& nh) {
    Configuration config;

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

    nh.param<std::string>("input_file",
        config.input_file,
        default_input_file);

    nh.param<std::string>("log_file",
        config.log_file,
        default_log_file);

    nh.param<std::string>("groundtruth_file",
        config.groundtruth_file,
        default_groundtruth_file);

    std::vector<float> gt_transform_vector;
    nh.getParam("gt_transform", gt_transform_vector);
    if (nh.getParam("gt_transform", gt_transform_vector)) {
      for (size_t i = 0; i < std::sqrt(gt_transform_vector.size()); ++i) {
        for (size_t j = 0; j < std::sqrt(gt_transform_vector.size()); ++j) {
          config.gt_transform(i, j) = gt_transform_vector[i * 4 + j];
        }
      }
    } else {
      config.gt_transform = default_gt_transform;
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

    nh.param<bool>("coloured_voxels",
        config.coloured_voxels,
        default_coloured_voxels);

    nh.param<bool>("multi_resolution",
        config.multi_resolution,
        default_multi_resolution);

    nh.param<bool>("bayesian",
        config.bayesian,
        default_bayesian);

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
    ROS_INFO("  input_file:             \"%s\"",
        config.input_file.c_str());
    ROS_INFO("  log_file:               \"%s\"",
        config.log_file.c_str());
    ROS_INFO("  groundtruth_file:       \"%s\"",
        config.groundtruth_file.c_str());
    ROS_INFO("  gt_transform:");
    for (size_t i = 0; i < 4; ++i) {
      ROS_INFO("                          %f %f %f %f",
          config.gt_transform(4 * i + 0),
          config.gt_transform(4 * i + 1),
          config.gt_transform(4 * i + 2),
          config.gt_transform(4 * i + 3));
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
    ROS_INFO("  coloured_voxels:        %d",
        config.coloured_voxels);
    ROS_INFO("  multi_resolution:       %d",
        config.multi_resolution);
    ROS_INFO("  bayesian:               %d",
        config.bayesian);
  }

} // namespace se

