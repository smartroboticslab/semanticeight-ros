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
constexpr bool default_enable_objects = false;
const Eigen::Vector2i default_input_res(640, 480);
const std::string default_pose_topic_type("geometry_msgs::PoseStamped");
constexpr int default_pose_buffer_size = 600;
constexpr int default_depth_buffer_size = 60;
constexpr int default_rgb_buffer_size = 60;
constexpr double default_max_timestamp_diff = 0.001;
constexpr bool default_center_at_first_position = true;
constexpr int default_visualization_rate = 4;
constexpr float default_visualization_max_z = INFINITY;
const std::string default_dataset = "Matterport3D";
constexpr bool default_visualize_360_raycasting = false;
constexpr double default_max_exploration_time = std::nan("");
constexpr bool default_run_segmentation = false;
const std::string default_control_interface = "SRL";


namespace se {
SupereightNodeConfig read_supereight_node_config(const ros::NodeHandle& nh)
{
    SupereightNodeConfig config;

    nh.param<bool>(
        "supereight_ros/enable_tracking", config.enable_tracking, default_enable_tracking);

    nh.param<bool>(
        "supereight_ros/enable_rendering", config.enable_rendering, default_enable_rendering);

    nh.param<bool>("supereight_ros/enable_rgb", config.enable_rgb, default_enable_rgb);

    nh.param<bool>("supereight_ros/enable_objects", config.enable_objects, default_enable_objects);

    // Try to read the camera parameters from the habitat-ros config if it exists and if that fails
    // try the supereight-ros config.
    std::vector<int> input_res_vector;
    if (nh.getParam("habitat/width", config.input_res[0])
        && nh.getParam("habitat/height", config.input_res[1])) {
        // Value already set in the condition.
    }
    else if (nh.getParam("supereight_ros/input_res", input_res_vector)) {
        for (size_t i = 0; i < input_res_vector.size(); ++i) {
            config.input_res[i] = input_res_vector[i];
        }
    }
    else {
        config.input_res = default_input_res;
    }

    nh.param<std::string>(
        "supereight_ros/pose_topic_type", config.pose_topic_type, default_pose_topic_type);

    nh.param<int>(
        "supereight_ros/pose_buffer_size", config.pose_buffer_size, default_pose_buffer_size);

    nh.param<int>(
        "supereight_ros/depth_buffer_size", config.depth_buffer_size, default_depth_buffer_size);

    nh.param<int>(
        "supereight_ros/rgb_buffer_size", config.rgb_buffer_size, default_rgb_buffer_size);

    nh.param<double>(
        "supereight_ros/max_timestamp_diff", config.max_timestamp_diff, default_max_timestamp_diff);

    nh.param<bool>("supereight_ros/center_at_first_position",
                   config.center_at_first_position,
                   default_center_at_first_position);

    nh.param<int>(
        "supereight_ros/visualization_rate", config.visualization_rate, default_visualization_rate);

    nh.param<float>("supereight_ros/visualization_max_z",
                    config.visualization_max_z,
                    default_visualization_max_z);

    std::string dataset_str;
    nh.param<std::string>("supereight_ros/dataset", dataset_str, default_dataset);
    config.dataset = str_to_dataset(dataset_str);

    nh.param<bool>("supereight_ros/visualize_360_raycasting",
                   config.visualize_360_raycasting,
                   default_visualize_360_raycasting);

    nh.param<double>("supereight_ros/max_exploration_time",
                     config.max_exploration_time,
                     default_max_exploration_time);

    nh.param<bool>(
        "supereight_ros/run_segmentation", config.run_segmentation, default_run_segmentation);

    std::string control_interface_str;
    nh.param<std::string>(
        "supereight_ros/control_interface", control_interface_str, default_control_interface);
    config.control_interface = str_to_control_interface(control_interface_str);

    // Can't have objects or semantics with RGB disabled.
    if (!config.enable_rgb && config.run_segmentation) {
        config.run_segmentation = false;
        ROS_WARN("Disabling object segmentation since RGB is disabled");
    }
    if (!config.enable_rgb && config.enable_objects) {
        config.enable_objects = false;
        ROS_WARN("Disabling object reconstruction since RGB is disabled");
    }

    return config;
}



void print_supereight_node_config(const SupereightNodeConfig& config)
{
    ROS_INFO("Supereight Node parameters:");
    ROS_INFO("  enable_tracking:          %d", config.enable_tracking);
    ROS_INFO("  enable_rendering:         %d", config.enable_rendering);
    ROS_INFO("  enable_rgb:               %d", config.enable_rgb);
    ROS_INFO("  enable_objects:           %d", config.enable_objects);
    ROS_INFO("  input_res:                %d %d", config.input_res.x(), config.input_res.y());
    ROS_INFO("  pose_topic_type:          %s", config.pose_topic_type.c_str());
    ROS_INFO("  pose_buffer_size:         %d", config.pose_buffer_size);
    ROS_INFO("  depth_buffer_size:        %d", config.depth_buffer_size);
    ROS_INFO("  rgb_buffer_size:          %d", config.rgb_buffer_size);
    ROS_INFO("  max_timestamp_diff:       %f", config.max_timestamp_diff);
    ROS_INFO("  center_at_first_position: %d", config.center_at_first_position);
    ROS_INFO("  visualization_rate:       %d", config.visualization_rate);
    ROS_INFO("  visualization_max_z:      %f", config.visualization_max_z);
    ROS_INFO("  dataset:                  %s", dataset_to_str(config.dataset).c_str());
    ROS_INFO("  control_interface:        %s",
             control_interface_to_str(config.control_interface).c_str());
}



Configuration read_supereight_config(const ros::NodeHandle& nh)
{
    Configuration config;

    // General
    nh.getParam("supereight/general/sequence_name", config.sequence_name);
    nh.getParam("supereight/general/sequence_type", config.sequence_type);
    bool enable_tracking = false;
    nh.getParam("supereight_ros/enable_tracking", enable_tracking);
    config.enable_ground_truth = !enable_tracking;
    nh.getParam("supereight_ros/enable_rendering", config.enable_render);
    nh.getParam("supereight/general/log_path", config.log_path);
    config.log_path = str_utils::expand_user(config.log_path);
    nh.getParam("supereight/general/output_render_path", config.output_render_file);
    config.output_render_file = str_utils::expand_user(config.output_render_file);
    nh.getParam("supereight/general/enable_meshing", config.enable_meshing);
    nh.getParam("supereight/general/output_mesh_path", config.output_mesh_file);
    config.output_mesh_file = str_utils::expand_user(config.output_mesh_file);
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
        if (!se::math::is_valid_transformation(config.T_BC)) {
            ROS_FATAL("Supplied T_BC is not a homogeneous transformation");
            abort();
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
        if (!se::math::is_valid_transformation(config.init_T_WB)) {
            ROS_FATAL("Supplied init_T_WB is not a homogeneous transformation");
            abort();
        }
    }

    nh.getParam("supereight/sensor/near_plane", config.near_plane);

    nh.getParam("supereight/sensor/far_plane", config.far_plane);

    // Voxel Impl
    config.voxel_impl_type = VoxelImpl::type();
    // TODO load the voxel implementation parameters from the YAML file.
    // Couldn't find a way to get the filename of the YAML file loaded from ROS.

    // Exploration only ///////////////////////////////////////////////////////
    nh.getParam("supereight/map/frontier_cluster_min_ratio", config.frontier_cluster_min_ratio);
    std::vector<float> aabb_min_W_vector;
    if (nh.getParam("supereight/map/aabb_min_W", aabb_min_W_vector)) {
        for (size_t i = 0; i < aabb_min_W_vector.size(); ++i) {
            config.aabb_min_W[i] = aabb_min_W_vector[i];
        }
    }
    std::vector<float> aabb_max_W_vector;
    if (nh.getParam("supereight/map/aabb_max_W", aabb_max_W_vector)) {
        for (size_t i = 0; i < aabb_max_W_vector.size(); ++i) {
            config.aabb_max_W[i] = aabb_max_W_vector[i];
        }
    }
    std::vector<float> sampling_min_W_vector;
    if (nh.getParam("supereight/map/sampling_min_W", sampling_min_W_vector)) {
        for (size_t i = 0; i < sampling_min_W_vector.size(); ++i) {
            config.sampling_min_W[i] = sampling_min_W_vector[i];
        }
    }
    std::vector<float> sampling_max_W_vector;
    if (nh.getParam("supereight/map/sampling_max_W", sampling_max_W_vector)) {
        for (size_t i = 0; i < sampling_max_W_vector.size(); ++i) {
            config.sampling_max_W[i] = sampling_max_W_vector[i];
        }
    }
    nh.getParam("supereight/exploration/enable_exploration", config.enable_exploration);
    nh.getParam("supereight/exploration/num_candidates", config.num_candidates);
    nh.getParam("supereight/exploration/frontier_sampling_probability",
                config.frontier_sampling_probability);
    std::vector<float> utility_weights_vector;
    if (nh.getParam("supereight/exploration/utility_weights", utility_weights_vector)) {
        config.utility_weights = Eigen::VectorXf(utility_weights_vector.size());
        for (size_t i = 0; i < utility_weights_vector.size(); ++i) {
            config.utility_weights[i] = utility_weights_vector[i];
        }
    }
    nh.getParam("supereight/exploration/use_pose_history", config.use_pose_history);
    nh.getParam("supereight/exploration/raycast_width", config.raycast_width);
    nh.getParam("supereight/exploration/raycast_height", config.raycast_height);
    nh.getParam("supereight/exploration/linear_velocity", config.linear_velocity);
    nh.getParam("supereight/exploration/angular_velocity", config.angular_velocity);
    nh.getParam("supereight/exploration/delta_t", config.delta_t);
    nh.getParam("supereight/exploration/robot_radius", config.robot_radius);
    nh.getParam("supereight/exploration/skeleton_sample_precision",
                config.skeleton_sample_precision);
    nh.getParam("supereight/exploration/solving_time", config.solving_time);
    nh.getParam("supereight/exploration/goal_xy_threshold", config.goal_xy_threshold);
    nh.getParam("supereight/exploration/goal_z_threshold", config.goal_z_threshold);
    nh.getParam("supereight/exploration/goal_roll_pitch_threshold",
                config.goal_roll_pitch_threshold);
    nh.getParam("supereight/exploration/goal_yaw_threshold", config.goal_yaw_threshold);

    return config;
}

} // namespace se
