// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros.hpp"

#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <map>
#include <thread>

#include <lodepng.h>

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include "supereight_ros/filesystem.hpp"
#include "supereight_ros/utilities.hpp"
#include "se/voxel_implementations.hpp"



auto get_image_timestamp = [](sensor_msgs::ImageConstPtr img) { return img->header.stamp; };

namespace se {
  // TODO SEM No idea why this needs to be here
  std::ostream& operator<<(std::ostream& out, const se::Configuration& config) {

    out << str_utils::header_to_pretty_str("GENERAL") << "\n";
    out << str_utils::str_to_pretty_str(config.voxel_impl_type,         "Voxel impl type") << "\n";
    out << str_utils::str_to_pretty_str(config.sensor_type,             "Sensor type") << "\n";
    out << "\n";

    out << str_utils::str_to_pretty_str(config.sequence_name,           "Sequence name") << "\n";
    out << str_utils::str_to_pretty_str(config.sequence_type,           "Sequence type") << "\n";
    out << str_utils::str_to_pretty_str(config.sequence_path,           "Sequence path") << "\n";
    out << str_utils::str_to_pretty_str(config.ground_truth_file,       "Ground truth file") << "\n";
    out << str_utils::str_to_pretty_str((config.log_file == "" ? "std::cout" : config.log_file),
                                                                        "Log file") << "\n";
    out << str_utils::bool_to_pretty_str(config.enable_benchmark,       "Enable benchmark") << "\n";
    out << str_utils::bool_to_pretty_str(config.enable_ground_truth,    "Enable ground truth") << "\n";
    out << str_utils::bool_to_pretty_str(config.enable_render,          "Enable render"      ) << "\n";
    if (config.output_render_file != "") {
      out << str_utils::str_to_pretty_str(config.output_render_file,    "Output render file") << "\n";
    }
    out << str_utils::bool_to_pretty_str(config.enable_meshing,         "Enable meshing"     ) << "\n";
    if (config.output_mesh_file != "") {
      out << str_utils::str_to_pretty_str(config.output_mesh_file,      "Output mesh file") << "\n";
    }
    out << str_utils::bool_to_pretty_str(config.enable_structure,       "Enable structure"     ) << "\n";
    if (config.output_structure_file != "") {
      out << str_utils::str_to_pretty_str(config.output_structure_file, "Output structure file") << "\n";
    }
    out << "\n";

    out << str_utils::value_to_pretty_str(config.integration_rate,      "Integration rate") << "\n";
    out << str_utils::value_to_pretty_str(config.rendering_rate,        "Rendering rate") << "\n";
    out << str_utils::value_to_pretty_str(config.meshing_rate,          "Meshing rate") << "\n";
    out << str_utils::value_to_pretty_str(config.fps,                   "FPS") << "\n";
    out << str_utils::bool_to_pretty_str(config.drop_frames,            "Drop frames") << "\n";
    out << str_utils::value_to_pretty_str(config.max_frame,             "Max frame") << "\n";
    out << "\n";

    out << str_utils::vector_to_pretty_str(Eigen::VectorXi::Map(config.pyramid.data(), config.pyramid.size()),
                                                                        "ICP pyramid levels") << "\n";
    out << str_utils::value_to_pretty_str(config.icp_threshold,         "ICP threshold") << "\n";
    out << str_utils::bool_to_pretty_str(config.render_volume_fullsize, "Render volume full-size") << "\n";
    out << "\n";

    out << str_utils::header_to_pretty_str("MAP") << "\n";
    out << str_utils::volume_to_pretty_str(config.map_size,             "Map size", "voxel") << "\n";
    out << str_utils::volume_to_pretty_str(config.map_dim,              "Map dim",  "meter") << "\n";
    out << str_utils::value_to_pretty_str(config.map_dim.x() / config.map_size.x(),
                                                                        "Map res", "meter/voxel") << "\n";

    out << str_utils::vector_to_pretty_str(config.t_MW_factor,          "t_MW_factor") << "\n";
    out << "\n";

    out << str_utils::header_to_pretty_str("SENSOR") << "\n";
    out << str_utils::vector_to_pretty_str(config.sensor_intrinsics,    "Sensor intrinsics", {"fx", "fy", "cx", "cy"}) << "\n";
    out << str_utils::bool_to_pretty_str(config.left_hand_frame,        "Left-handed-coordinate system") << "\n";
    out << str_utils::value_to_pretty_str(config.sensor_downsampling_factor,
                                                                        "Sensor downsampling factor") << "\n";
    out << str_utils::bool_to_pretty_str(config.bilateral_filter,       "Filter depth (bilateral filter)") << "\n";
    out << str_utils::value_to_pretty_str(config.near_plane,            "Near plane", "meters") << "\n";
    out << str_utils::value_to_pretty_str(config.far_plane,             "Far plane", "meters") << "\n";
    out << "\n";
    out << str_utils::matrix_to_pretty_str(config.T_BC,                 "T_BC") << "\n";
    out << "\n";
    out << str_utils::matrix_to_pretty_str(config.init_T_WB,            "init_T_WB") << "\n";
    out << "\n";

    const Eigen::Vector3f t_MW = config.map_dim.x() * config.t_MW_factor;
    const Eigen::Matrix4f T_MW = se::math::to_transformation(t_MW);
    const Eigen::Matrix4f init_T_MB = T_MW * config.init_T_WB;
    const Eigen::Vector3f init_t_MB = se::math::to_translation(init_T_MB);
    out << str_utils::vector_to_pretty_str(init_t_MB,                   "init t_MB") << "\n";
    out << "\n";

    const Eigen::Vector3f init_t_MB_factor = init_t_MB / config.map_dim.x();
    out << str_utils::vector_to_pretty_str(init_t_MB_factor,            "init t_MB_factor") << "\n";
    out << "\n";

    // Exploration only ///////////////////////////////////////////////////////
    out << str_utils::vector_to_pretty_str(config.aabb_min_W,               "AABB min_w") << "\n";
    out << str_utils::vector_to_pretty_str(config.aabb_max_W,               "AABB max_w") << "\n";
    out << str_utils::bool_to_pretty_str(config.enable_exploration,         "Enable exploration") << "\n";
    out << str_utils::value_to_pretty_str(config.num_candidates,            "Num candidates") << "\n";
    out << str_utils::value_to_pretty_str(config.exploration_weight,        "Exploration weight") << "\n";
    out << str_utils::bool_to_pretty_str(config.use_pose_history,           "Use pose history") << "\n";
    out << str_utils::value_to_pretty_str(config.raycast_width,             "Raycast width") << "\n";
    out << str_utils::value_to_pretty_str(config.raycast_height,            "Raycast height") << "\n";
    out << str_utils::value_to_pretty_str(config.linear_velocity,           "Linear velocity") << "\n";
    out << str_utils::value_to_pretty_str(config.angular_velocity,          "Angular velocity") << "\n";
    out << str_utils::value_to_pretty_str(config.delta_t,                   "Delta t") << "\n";
    out << str_utils::value_to_pretty_str(config.robot_radius,              "Robot radius") <<  "\n";
    out << str_utils::value_to_pretty_str(config.safety_radius,             "Safety radius") << "\n";
    out << str_utils::value_to_pretty_str(config.min_control_point_radius,  "Min control point radius") << "\n";
    out << str_utils::value_to_pretty_str(config.skeleton_sample_precision, "Skeleton sample precision") << "\n";
    out << str_utils::value_to_pretty_str(config.solving_time,              "Solving time") << "\n";

    return out;
  }



SupereightNode::SupereightNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      sensor_({1, 1, false, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}),
      frame_(0),
      num_planning_iterations_(0),
      num_failed_planning_iterations_(0),
      max_failed_planning_iterations_(0),
      input_segmentation_(0, 0),
#ifdef SE_WITH_MASKRCNN
      network_(network_config_),
#endif // SE_WITH_MASKRCNN
      world_frame_id_("world"),
      map_frame_id_("map"),
      body_frame_id_("body"),
      camera_frame_id_("camera") {

  readConfig(nh_private);
  if (node_config_.experiment_type == "gazebo") {
    body_frame_id_ = "firefly/base_link";
    camera_frame_id_ = "firefly/vi_sensor/camera_depth_optical_center_link";
  }

  t_MW_ = supereight_config_.t_MW_factor.cwiseProduct(supereight_config_.map_dim);
  T_WM_ = Eigen::Matrix4f::Identity();
  T_WM_.topRightCorner<3,1>() = -t_MW_;
  T_CB_ = supereight_config_.T_BC.inverse();
  init_t_WB_ = Eigen::Vector3f::Constant(NAN);
  image_res_ = node_config_.input_res / supereight_config_.sensor_downsampling_factor;

  // Allocate input image buffers.
  const size_t input_num_pixels = node_config_.input_res.prod();
  input_depth_ = std::unique_ptr<float>(new float[input_num_pixels]);
  if (node_config_.enable_rgb) {
    input_rgba_ = std::unique_ptr<uint32_t>(new uint32_t[input_num_pixels]);
  }

  // Allocate rendered image buffers.
  if (node_config_.enable_rendering) {
    const size_t render_num_pixels = image_res_.prod();
    depth_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    if (node_config_.enable_rgb) {
      rgba_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    }
    if (node_config_.enable_tracking) {
      track_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    }
    volume_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    volume_render_color_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    volume_render_scale_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    volume_render_min_scale_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    if (node_config_.enable_objects) {
      class_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
      instance_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
    }
    raycast_render_ = std::unique_ptr<uint32_t>(new uint32_t[render_num_pixels]);
  }

  // Initialize the sensor.
  const Eigen::Vector2i downsampled_res = node_config_.input_res / supereight_config_.sensor_downsampling_factor;
  const Eigen::VectorXf elevation_angles = (Eigen::VectorXf(64) <<
      17.74400, 17.12000, 16.53600, 15.98200, 15.53000, 14.93600, 14.37300, 13.82300,
      13.37300, 12.78600, 12.23000, 11.68700, 11.24100, 10.67000, 10.13200, 9.57400,
      9.13800, 8.57700, 8.02300, 7.47900, 7.04600, 6.48100, 5.94400, 5.39500, 4.96300,
      4.40100, 3.85900, 3.31900, 2.87100, 2.32400, 1.78300, 1.23800, 0.78600, 0.24500,
      -0.29900, -0.84900, -1.28800, -1.84100, -2.27500, -2.92600, -3.37800, -3.91000,
      -4.45700, -5.00400, -5.46000, -6.00200, -6.53700, -7.09600, -7.55200, -8.09000,
      -8.62900, -9.19600, -9.65700, -10.18300, -10.73200, -11.28900, -11.77000, -12.29700,
      -12.85400, -13.41500, -13.91600, -14.44200, -14.99700, -15.59500).finished();
  const Eigen::VectorXf azimuth_angles = (Eigen::VectorXf(64) <<
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0).finished();
  sensor_ = SensorImpl({downsampled_res.x(), downsampled_res.y(),
      supereight_config_.left_hand_frame, supereight_config_.near_plane, supereight_config_.far_plane,
      supereight_config_.sensor_intrinsics[0] / supereight_config_.sensor_downsampling_factor,
      supereight_config_.sensor_intrinsics[1] / supereight_config_.sensor_downsampling_factor,
      supereight_config_.sensor_intrinsics[2] / supereight_config_.sensor_downsampling_factor,
      supereight_config_.sensor_intrinsics[3] / supereight_config_.sensor_downsampling_factor,
      azimuth_angles, elevation_angles});

  // Initialize the supereight pipeline.
  pipeline_ = std::shared_ptr<DenseSLAMSystem>(new DenseSLAMSystem(
      image_res_,
      Eigen::Vector3i::Constant(supereight_config_.map_size.x()),
      Eigen::Vector3f::Constant(supereight_config_.map_dim.x()),
      t_MW_,
      supereight_config_.pyramid,
      supereight_config_));
  se::ExplorationConfig exploration_config = {
    supereight_config_.num_candidates, {
      supereight_config_.exploration_weight,
      supereight_config_.use_pose_history,
      supereight_config_.raycast_width,
      supereight_config_.raycast_height,
      supereight_config_.delta_t,
      supereight_config_.linear_velocity,
      supereight_config_.angular_velocity, {
        "", Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        supereight_config_.robot_radius,
        supereight_config_.safety_radius,
        supereight_config_.min_control_point_radius,
        supereight_config_.skeleton_sample_precision,
        supereight_config_.solving_time}}};
  planner_ = std::unique_ptr<se::ExplorationPlanner>(new se::ExplorationPlanner(
        pipeline_->getMap(), pipeline_->T_MW(), supereight_config_.T_BC, exploration_config));

  // Allocate message circular buffers.
  if (node_config_.enable_tracking) {
    pose_buffer_.set_capacity(0);
  } else {
    pose_buffer_.set_capacity(node_config_.pose_buffer_size);
  }
  depth_buffer_.set_capacity(node_config_.depth_buffer_size);
  if (node_config_.enable_rgb) {
    rgb_buffer_.set_capacity(node_config_.rgb_buffer_size);
    if (!node_config_.run_segmentation && node_config_.enable_objects) {
      class_buffer_.set_capacity(node_config_.rgb_buffer_size);
      instance_buffer_.set_capacity(node_config_.rgb_buffer_size);
    }
  } else {
    rgb_buffer_.set_capacity(0);
    class_buffer_.set_capacity(0);
    instance_buffer_.set_capacity(0);
  }

  // Use the correct classes.
  if (node_config_.run_segmentation) {
    se::use_coco_classes();
  } else {
    se::use_matterport3d_classes();
  }
  se::set_thing("chair");
  se::set_stuff("book");

  if (node_config_.run_segmentation) {
#ifdef SE_WITH_MASKRCNN
    network_config_.model_filename = "/home/srl/Documents/Datasets/MaskRCNN/mrcnn_nchw.uff";
    network_config_.serialized_model_filename = network_config_.model_filename + ".bin";
    network_ = mr::MaskRCNNConfig(network_config_);
    if (!network_.build()) {
        ROS_FATAL("Couldn't initialize the network.");
        abort();
    }
#endif // SE_WITH_MASKRCNN
  }

  setupRos();

  // Start the planner thread.
  if (supereight_config_.enable_exploration) {
    std::thread t (std::bind(&SupereightNode::plan, this));
    t.detach();
  }

  exploration_start_time_ = std::chrono::steady_clock::now();
  ROS_INFO("Initialization finished");

  if (node_config_.experiment_type == "gazebo") {
    for (double i = 0.0; i <= 0.2; i += 0.05) {
      mav_msgs::EigenTrajectoryPoint point;
      nh.param<double>("wp_x", point.position_W.x(), 0.0);
      nh.param<double>("wp_y", point.position_W.y(), 0.0);
      nh.param<double>("wp_z", point.position_W.z(), 1.0);
      // Take the constant tracking error of the controller into account
      point.position_W.z() += 0.2;
      tf::Quaternion q_tf_WB = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
      double yaw = tf::getYaw(q_tf_WB);
      point.setFromYaw(yaw);

      trajectory_msgs::MultiDOFJointTrajectory path_msg;
      path_msg.header.stamp = ros::Time::now();
      path_msg.points.clear();

      trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(point, &point_msg);
      path_msg.points.push_back(point_msg);
      path_pub_.publish(path_msg);

      ros::Duration(1.0).sleep();
    }
  }
}



void SupereightNode::matchAndFuse() {
  // matchAndFuse() should only be run by a single thread at a time. Return
  // if the lock can't be acquired (another thread is already running).
  std::unique_lock<std::mutex> matching_lock (matching_mutex_, std::defer_lock_t());
  if (!matching_lock.try_lock()) {
    return;
  }

  std::chrono::time_point<std::chrono::steady_clock> start_time;
  std::chrono::time_point<std::chrono::steady_clock> end_time;
  start_time = std::chrono::steady_clock::now();



  // Message association
  // Depth
  sensor_msgs::ImageConstPtr current_depth_msg;
  double depth_timestamp;
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    if (depth_buffer_.empty()) {
      return;
    } else {
      current_depth_msg = depth_buffer_.front();
      depth_timestamp = ros::Time(current_depth_msg->header.stamp).toSec();
    }
  }

  // RGB
  sensor_msgs::ImageConstPtr current_rgb_msg;
  if (node_config_.enable_rgb) {
    const std::lock_guard<std::mutex> rgb_lock (rgb_buffer_mutex_);
    if (rgb_buffer_.empty()) {
      return;
    } else {
      const bool found = get_closest_element(rgb_buffer_, depth_timestamp,
          node_config_.max_timestamp_diff, get_image_timestamp, current_rgb_msg);
      if (!found) {
        return;
      }
    }
  }

  bool semantics_found = true;
  // Class
  sensor_msgs::ImageConstPtr current_class_msg;
  if (node_config_.enable_objects) {
    const std::lock_guard<std::mutex> class_lock (class_buffer_mutex_);
    if (class_buffer_.empty()) {
      semantics_found = false;
    } else {
      const bool found = get_closest_element(class_buffer_, depth_timestamp,
          node_config_.max_timestamp_diff, get_image_timestamp, current_class_msg);
      if (!found) {
        semantics_found = false;
        ROS_WARN_ONCE("No matching semantic class images found");
      }
    }
  }

  // Instance
  sensor_msgs::ImageConstPtr current_instance_msg;
  if (node_config_.enable_objects) {
    const std::lock_guard<std::mutex> instance_lock (instance_buffer_mutex_);
    if (instance_buffer_.empty()) {
      semantics_found = false;
    } else {
      const bool found = get_closest_element(instance_buffer_, depth_timestamp,
          node_config_.max_timestamp_diff, get_image_timestamp, current_instance_msg);
      if (!found) {
        semantics_found = false;
        ROS_WARN_ONCE("No matching semantic instance images found");
      }
    }
  }

  // Pose
  Eigen::Matrix4f external_T_WC;
  if (!node_config_.enable_tracking) {
    const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
    if (pose_buffer_.empty()) {
      // Clear the depth and RGB buffers if no poses have arrived yet. These
      // images will never be associated to poses.
      depth_buffer_.clear();
      rgb_buffer_.clear(); // OK to call even when RGB images are not used
      class_buffer_.clear(); // OK to call even when class images are not used
      instance_buffer_.clear(); // OK to call even when instance images are not used
      return;
    } else {
      // Find the two closest poses and interpolate to the depth timestamp.
      geometry_msgs::TransformStamped prev_pose;
      geometry_msgs::TransformStamped next_pose;
      const InterpResult result = get_surrounding_poses(
          pose_buffer_, depth_timestamp, prev_pose, next_pose);
      if (result == InterpResult::query_smaller) {
        // Remove the depth image, it will never be matched to poses.
        const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
        depth_buffer_.pop_front();
        return;
      } else if (result == InterpResult::query_greater) {
        // Remove the first poses, they will never be matched to depth images.
        pose_buffer_.erase_begin(pose_buffer_.size() - 1);
        return;
      }

      // Interpolate to associate a pose to the depth image.
      external_T_WC = interpolate_pose(prev_pose, next_pose, depth_timestamp);
    }
  }

  // Copy the semantics into the appropriate buffer.
  if (node_config_.enable_objects) {
    if (semantics_found) {
      input_segmentation_ = to_supereight_segmentation(current_class_msg, current_instance_msg);
    } else {
      input_segmentation_ = se::SegmentationResult(0, 0);
    }
  }

  // The currect depth image is going to be integrated, remove it from the
  // buffer to avoid integrating it again.
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    depth_buffer_.pop_front();
  }
  end_time = std::chrono::steady_clock::now();
  times_matching_.push_back(std::chrono::duration<double>(end_time - start_time).count());

  if (node_config_.run_segmentation) {
    // If the network_mutex_ can be locked it means there is no thread running runNetwork().
    if (network_mutex_.try_lock()) {
      // Release the lock so that it can be acquired in runNetwork().
      network_mutex_.unlock();
      // Run the network in a background thread.
      std::thread t (std::bind(&SupereightNode::runNetwork, this, external_T_WC, current_depth_msg, current_rgb_msg));
      t.detach();
      // Don't call fuse with this depth frame as it will be done by runNetwork();
      return;
    }
  }

  // Call fuse() if runNetwork() wasn't called.
  fuse(external_T_WC, current_depth_msg, current_rgb_msg, input_segmentation_);
}



void SupereightNode::fuse(const Eigen::Matrix4f&            T_WC,
                          const sensor_msgs::ImageConstPtr& depth_image,
                          const sensor_msgs::ImageConstPtr& color_image,
                          const se::SegmentationResult&     segmentation) {
  const std::lock_guard<std::mutex> fusion_lock (fusion_mutex_);

  ROS_INFO("-----------------------------------------");
  ROS_INFO("Frame %d", frame_);

  std::chrono::time_point<std::chrono::steady_clock> start_time;
  std::chrono::time_point<std::chrono::steady_clock> end_time;

  // Convert the depth and RGB images into a format that supereight likes.
  to_supereight_depth(depth_image, sensor_.far_plane, input_depth_.get());
  if (node_config_.enable_rgb) {
    to_supereight_RGB(color_image, input_rgba_.get());
  }

  // Preprocessing
  start_time = std::chrono::steady_clock::now();
  pipeline_->preprocessDepth(input_depth_.get(), node_config_.input_res,
      supereight_config_.bilateral_filter);
  if (node_config_.enable_rgb) {
    pipeline_->preprocessColor(input_rgba_.get(), node_config_.input_res);
  }
  if (node_config_.enable_objects) {
    pipeline_->preprocessSegmentation(segmentation);
  }
  end_time = std::chrono::steady_clock::now();
  times_preprocessing_.push_back(std::chrono::duration<double>(end_time - start_time).count());

  // Tracking
  start_time = std::chrono::steady_clock::now();
  bool tracked = false;
  if (node_config_.enable_tracking) {
    if (frame_ % supereight_config_.tracking_rate == 0) {
      tracked = pipeline_->track(sensor_, supereight_config_.icp_threshold);
    } else {
      tracked = false;
    }
  } else {
    pipeline_->setT_WC(T_WC);
    tracked = true;
  }
  // Call object tracking.
  if (node_config_.enable_objects) {
    pipeline_->trackObjects(sensor_, frame_);
  }
  // Publish the pose estimated/received by supereight.
  const Eigen::Matrix4f se_T_WB = pipeline_->T_WC() * T_CB_;
  const Eigen::Vector3d se_t_WB = se_T_WB.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond se_q_WB (se_T_WB.block<3, 3>(0, 0).cast<double>());
  geometry_msgs::PoseStamped se_T_WB_msg;
  se_T_WB_msg.header = depth_image->header;
  se_T_WB_msg.header.frame_id = world_frame_id_;
  tf::pointEigenToMsg(se_t_WB, se_T_WB_msg.pose.position);
  tf::quaternionEigenToMsg(se_q_WB, se_T_WB_msg.pose.orientation);
  supereight_pose_pub_.publish(se_T_WB_msg);
  end_time = std::chrono::steady_clock::now();
  times_tracking_.push_back(std::chrono::duration<double>(end_time - start_time).count());

  // Integration
  // Integrate only if tracking was successful or it is one of the first 4
  // frames.
  bool integrated = false;
  if ((tracked && (frame_ % supereight_config_.integration_rate == 0)) || frame_ <= 3) {
    {
      const std::lock_guard<std::mutex> map_lock (map_mutex_);

      start_time = std::chrono::steady_clock::now();
      integrated = pipeline_->integrate(sensor_, frame_);
      end_time = std::chrono::steady_clock::now();
      times_integration_.push_back(std::chrono::duration<double>(end_time - start_time).count());

      start_time = std::chrono::steady_clock::now();
      if (node_config_.enable_objects) {
        integrated = pipeline_->integrateObjects(sensor_, frame_);
      }
      end_time = std::chrono::steady_clock::now();
      times_object_integration_.push_back(std::chrono::duration<double>(end_time - start_time).count());
    }

    {
      const std::lock_guard<std::mutex> map_lock (pose_mutex_);
      planner_->setT_WB(se_T_WB);
    }
  } else {
    integrated = false;
    times_integration_.push_back(0);
    times_object_integration_.push_back(0);
  }

  // Rendering
  start_time = std::chrono::steady_clock::now();
  if (node_config_.enable_rendering) {
    // Depth
    pipeline_->renderDepth(depth_render_.get(), image_res_, sensor_);
    const sensor_msgs::Image depth_render_msg = RGBA_to_msg(depth_render_.get(),
        image_res_, depth_image->header);
    depth_render_pub_.publish(depth_render_msg);

    // RGB
    if (node_config_.enable_rgb) {
      pipeline_->renderRGBA(rgba_render_.get(), image_res_);
      const sensor_msgs::Image rgba_render_msg = RGBA_to_msg(rgba_render_.get(), image_res_,
          depth_image->header);
      rgba_render_pub_.publish(rgba_render_msg);
    }

    // Track
    if (node_config_.enable_tracking) {
      pipeline_->renderTrack(track_render_.get(), image_res_);
      const sensor_msgs::Image track_render_msg = RGBA_to_msg(track_render_.get(), image_res_,
          depth_image->header);
      track_render_pub_.publish(track_render_msg);
    }

    // Volume
    if (frame_ % supereight_config_.rendering_rate == 0) {
      (void) pipeline_->raycastObjectsAndBg(sensor_, frame_);

      pipeline_->renderObjects(volume_render_.get(), image_res_, sensor_, RenderMode::InstanceID, false);
      volume_render_pub_.publish(RGBA_to_msg(volume_render_.get(), image_res_, depth_image->header));

      pipeline_->renderObjects(volume_render_color_.get(), image_res_, sensor_, RenderMode::Color, false);
      volume_render_color_pub_.publish(RGBA_to_msg(volume_render_color_.get(), image_res_, depth_image->header));

      pipeline_->renderObjects(volume_render_scale_.get(), image_res_, sensor_, RenderMode::Scale, false);
      volume_render_scale_pub_.publish(RGBA_to_msg(volume_render_scale_.get(), image_res_, depth_image->header));

      pipeline_->renderObjects(volume_render_min_scale_.get(), image_res_, sensor_, RenderMode::MinScale, false);
      volume_render_min_scale_pub_.publish(RGBA_to_msg(volume_render_min_scale_.get(), image_res_, depth_image->header));

      if (node_config_.enable_objects) {
        pipeline_->renderObjectClasses(class_render_.get(), image_res_);
        class_render_pub_.publish(RGBA_to_msg(class_render_.get(), image_res_, depth_image->header));

        pipeline_->renderObjectInstances(instance_render_.get(), image_res_);
        instance_render_pub_.publish(RGBA_to_msg(instance_render_.get(), image_res_, depth_image->header));
      }

      pipeline_->renderRaycast(raycast_render_.get(), image_res_);
      raycast_render_pub_.publish(RGBA_to_msg(raycast_render_.get(), image_res_, depth_image->header));
    }

    if (node_config_.visualize_360_raycasting) {
      // Entropy
      const se::Image<uint32_t> entropy_render = planner_->renderCurrentEntropy(sensor_);
      const sensor_msgs::Image entropy_render_msg = RGBA_to_msg(entropy_render.data(),
          Eigen::Vector2i(entropy_render.width(), entropy_render.height()), depth_image->header);
      entropy_render_pub_.publish(entropy_render_msg);

      // Entropy depth
      const se::Image<uint32_t> entropy_depth_render = planner_->renderCurrentEntropyDepth(sensor_);
      const sensor_msgs::Image entropy_depth_render_msg = RGBA_to_msg(entropy_depth_render.data(),
          Eigen::Vector2i(entropy_depth_render.width(), entropy_depth_render.height()),
          depth_image->header);
      entropy_depth_render_pub_.publish(entropy_depth_render_msg);
    }
  }
  end_time = std::chrono::steady_clock::now();
  times_rendering_.push_back(std::chrono::duration<double>(end_time - start_time).count());

  // Visualization
  start_time = std::chrono::steady_clock::now();
  map_dim_pub_.publish(map_dim_msg_);
  if (node_config_.visualization_rate > 0 && (frame_ % node_config_.visualization_rate == 0)) {
    visualizeWholeMap();
    visualizeMapMesh();
    if (node_config_.enable_objects) {
      //visualizeObjects();
      visualizeObjectMeshes();
      visualizeObjectAABBs();
    }
    visualizeFrontiers();
    visualizePoseHistory();
  }
  end_time = std::chrono::steady_clock::now();
  times_visualization_.push_back(std::chrono::duration<double>(end_time - start_time).count());

  printFrameTimes();
  ROS_INFO("Free volume:     %10.3f m^3", pipeline_->free_volume);
  ROS_INFO("Occupied volume: %10.3f m^3", pipeline_->occupied_volume);
  ROS_INFO("Explored volume: %10.3f m^3", pipeline_->explored_volume);
  ROS_INFO("Tracked: %d   Integrated: %d", tracked, integrated);

  if (supereight_config_.rendering_rate > 0 && (frame_ + 1) % supereight_config_.rendering_rate == 0 && supereight_config_.output_render_file != "") {
    stdfs::create_directories(supereight_config_.output_render_file);

    const int w = (pipeline_->getImageResolution()).x();
    const int h = (pipeline_->getImageResolution()).y();
    const std::string prefix = supereight_config_.output_render_file + "/";
    std::stringstream path_suffix_ss;
    path_suffix_ss << std::setw(5) << std::setfill('0') << frame_ << ".png";
    const std::string suffix = path_suffix_ss.str();

    std::unique_ptr<uint32_t[]> segmentation_render (new uint32_t[w * h]);
    pipeline_->renderInputSegmentation(segmentation_render.get(), pipeline_->getImageResolution());
    std::unique_ptr<uint32_t[]> volume_aabb_render (new uint32_t[w * h]);
    pipeline_->renderObjects(volume_aabb_render.get(), pipeline_->getImageResolution(), sensor_, RenderMode::InstanceID);

    lodepng_encode32_file((prefix + "rgba_" + suffix).c_str(), (unsigned char*) rgba_render_.get(), w, h);
    lodepng_encode32_file((prefix + "depth_" + suffix).c_str(), (unsigned char*) depth_render_.get(), w, h);
    lodepng_encode32_file((prefix + "segm_" + suffix).c_str(), (unsigned char*) segmentation_render.get(), w, h);
    lodepng_encode32_file((prefix + "volume_" + suffix).c_str(), (unsigned char*) volume_render_.get(), w, h);
    lodepng_encode32_file((prefix + "volume_color_" + suffix).c_str(), (unsigned char*) volume_render_color_.get(), w, h);
    lodepng_encode32_file((prefix + "volume_scale_" + suffix).c_str(), (unsigned char*) volume_render_scale_.get(), w, h);
    lodepng_encode32_file((prefix + "volume_min_scale_" + suffix).c_str(), (unsigned char*) volume_render_min_scale_.get(), w, h);
    lodepng_encode32_file((prefix + "volume_aabb_" + suffix).c_str(), (unsigned char*) volume_aabb_render.get(), w, h);
    lodepng_encode32_file((prefix + "raycast_" + suffix).c_str(), (unsigned char*) raycast_render_.get(), w, h);
    if (node_config_.enable_objects) {
      lodepng_encode32_file((prefix + "instance_" + suffix).c_str(), (unsigned char*) instance_render_.get(), w, h);
      lodepng_encode32_file((prefix + "class_" + suffix).c_str(), (unsigned char*) class_render_.get(), w, h);
    }
  }

  if (supereight_config_.meshing_rate > 0 && (frame_ + 1) % supereight_config_.meshing_rate == 0) {
    SupereightNode::saveMap();
  }

  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - exploration_start_time_).count() > node_config_.max_exploration_time) {
    ROS_INFO("Reached time limit of %.3f s, stopping", node_config_.max_exploration_time);
    raise(SIGINT);
  }
  frame_++;
}



void SupereightNode::plan() {
  // Wait for the first pose
  while (true) {
    std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    const std::lock_guard<std::mutex> pose_lock (pose_mutex_);
    if (!planner_->getT_WBHistory().empty()) {
      break;
    }
  }
  // Free the initial position to allow planning.
  {
    const std::lock_guard<std::mutex> map_lock (map_mutex_);
    pipeline_->freeInitialPosition(sensor_, (node_config_.experiment_type == "gazebo" ? "sphere" : "cylinder"));
  }
  // Exploration planning
  while (num_failed_planning_iterations_ < max_failed_planning_iterations_ || max_failed_planning_iterations_ == 0) {
    bool goal_reached = false;
    {
      const std::lock_guard<std::mutex> pose_lock (pose_mutex_);
      goal_reached = planner_->goalReached();
    }
    if (goal_reached || num_planning_iterations_ == 0) {
      if (planner_->needsNewGoal()) {
        se::Path path_WB;
        {
          const std::lock_guard<std::mutex> map_lock (map_mutex_);
          const std::lock_guard<std::mutex> pose_lock (pose_mutex_);
          const auto start_time = std::chrono::steady_clock::now();
          path_WB = planner_->computeNextPath_WB(pipeline_->getFrontiers(), pipeline_->getObjectMaps(), sensor_);
          const auto end_time = std::chrono::steady_clock::now();
          times_planning_.push_back(std::chrono::duration<double>(end_time - start_time).count());
        }
        ROS_WARN("Planning iteration %d", num_planning_iterations_);
        ROS_WARN("%-25s %.5f s", "Planning", times_planning_.back());

        if (path_WB.empty()) {
          num_failed_planning_iterations_++;
        } else {
          ROS_WARN("Planning %d, next goal is candidate %zu",
              num_planning_iterations_, planner_->goalViewIndex());
          for (size_t i = 0; i < planner_->candidateViews().size(); ++i) {
            ROS_WARN("Planning %d candidate %2zu utility: %s",
                num_planning_iterations_, i, planner_->candidateViews()[i].utilityStr().c_str());
          }
          for (size_t i = 0; i < planner_->candidateViews().size(); ++i) {
            const Eigen::Vector3f goal_t_WB =
              (T_WM_ * planner_->candidateViews()[i].goalT_MB().topRightCorner<4,1>()).head<3>();
            ROS_WARN("Planning %d candidate %2zu t_WB: % .3f % .3f % .3f",
                num_planning_iterations_, i, goal_t_WB.x(), goal_t_WB.y(), goal_t_WB.z());
          }
          if (node_config_.visualization_rate > 0) {
            visualizeCandidates();
            visualizeCandidatePaths();
            visualizeRejectedCandidates();
            visualizeGoal();
          }
          if (supereight_config_.rendering_rate > 0 && supereight_config_.output_render_file != "") {
            stdfs::create_directories(supereight_config_.output_render_file);
            std::stringstream prefix_ss;
            prefix_ss << supereight_config_.output_render_file << "/planning_" << std::setw(5) << std::setfill('0') << num_planning_iterations_ << "_";
            const std::string prefix = prefix_ss.str();

            const se::CandidateView& goal_view = planner_->goalView();
            const se::Image<uint32_t> entropy_render = goal_view.renderEntropy(sensor_);
            const se::Image<uint32_t> entropy_depth_render = planner_->renderCurrentEntropyDepth(sensor_);
            const se::Image<uint32_t> min_scale_render = planner_->renderMinScale(sensor_);
            lodepng_encode32_file((prefix + "goal_entropy.png").c_str(), (unsigned char*) entropy_render.data(), entropy_render.width(), entropy_render.height());
            lodepng_encode32_file((prefix + "goal_depth.png").c_str(), (unsigned char*) entropy_depth_render.data(), entropy_depth_render.width(), entropy_depth_render.height());
            lodepng_encode32_file((prefix + "goal_min_scale.png").c_str(), (unsigned char*) min_scale_render.data(), min_scale_render.width(), min_scale_render.height());
            // Visualize the individual candidates
            const auto& candidates = planner_->candidateViews();
            for (size_t i = 0; i < candidates.size(); ++i) {
              std::stringstream suffix_ss;
              suffix_ss << "candidate_" << std::setw(2) << std::setfill('0') << i << "_entropy_" << candidates[i].entropy_ << "_yaw_" << candidates[i].yaw_M_ << ".png";
              std::string suffix = suffix_ss.str();
              const se::Image<uint32_t> entropy_render = candidates[i].renderEntropy(sensor_);
              lodepng_encode32_file((prefix + suffix).c_str(), (unsigned char*) entropy_render.data(), entropy_render.width(), entropy_render.height());
              suffix.replace(suffix.size() - 3, 3, "txt");
              se::write_entropy(prefix + suffix, candidates[i].entropy_image_, sensor_);
            }
          }
          if (supereight_config_.output_mesh_file != "") {
            stdfs::create_directories(supereight_config_.output_mesh_file);
            std::stringstream filename_ss;
            filename_ss << supereight_config_.output_mesh_file << "/planning_" << std::setw(5) << std::setfill('0') << num_planning_iterations_ << "_goal_path_W.tsv";
            se::write_path_tsv(filename_ss.str(), path_WB);
          }
          saveCandidates();
        }
        num_planning_iterations_++;
      }
      // Change the path publishing method depending on the dataset type.
      if (node_config_.experiment_type == "gazebo") {
        publish_path_open_loop(*planner_, path_pub_, world_frame_id_, node_config_.experiment_type, supereight_config_.delta_t);
      } else {
        publish_path_vertex(*planner_, path_pub_, world_frame_id_, node_config_.experiment_type);
      }
    }
  }
  ROS_INFO("Failed to plan %d times, stopping", num_failed_planning_iterations_);
  raise(SIGINT);
}



void SupereightNode::saveMap() {
  if (supereight_config_.enable_meshing && !supereight_config_.output_mesh_file.empty()) {
    stdfs::create_directories(supereight_config_.output_mesh_file);
    std::stringstream output_mesh_meter_file_ss;
    output_mesh_meter_file_ss << supereight_config_.output_mesh_file << "/mesh_"
                              << std::setw(5) << std::setfill('0') << frame_ << ".ply";
    pipeline_->dumpMesh("", output_mesh_meter_file_ss.str().c_str());
    std::stringstream output_mesh_object_file_ss;
    output_mesh_object_file_ss << supereight_config_.output_mesh_file << "/mesh_"
                              << std::setw(5) << std::setfill('0') << frame_ << "_object";
    pipeline_->dumpObjectMeshes(output_mesh_object_file_ss.str().c_str(), false);
    std::stringstream output_path_ply_file_ss;
    output_path_ply_file_ss << supereight_config_.output_mesh_file << "/path_"
                              << std::setw(5) << std::setfill('0') << frame_ << ".ply";
    planner_->writePathPLY(output_path_ply_file_ss.str());
    std::stringstream output_path_tsv_file_ss;
    output_path_tsv_file_ss << supereight_config_.output_mesh_file << "/path_"
                              << std::setw(5) << std::setfill('0') << frame_ << ".tsv";
    planner_->writePathTSV(output_path_tsv_file_ss.str());
    ROS_INFO("Map saved in %s\n", supereight_config_.output_mesh_file.c_str());
  }
}



int SupereightNode::saveCandidates() {
  if (supereight_config_.enable_meshing && !supereight_config_.output_mesh_file.empty()) {
    stdfs::create_directories(supereight_config_.output_mesh_file);
    std::stringstream output_candidate_mesh_file_ss;
    output_candidate_mesh_file_ss << supereight_config_.output_mesh_file << "/candidates_"
        << std::setw(5) << std::setfill('0') << num_planning_iterations_ << ".ply";
    const std::string filename = output_candidate_mesh_file_ss.str();
    // Open file
    std::ofstream file (filename);
    if (!file.is_open()) {
      ROS_WARN("Unable to write file %s\n", filename.c_str());
      return 1;
    }
    // Write header
    const auto& candidates = planner_->candidateViews();
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "comment Candidate view positions\n";
    file << "comment Generated by semanticeight\n";
    file << "element vertex " << candidates.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";
    // Write candidate positions
    for (const auto& candidate : candidates) {
      const Eigen::Vector3f goal_t_WB =
          (T_WM_ * candidate.goalT_MB().topRightCorner<4,1>()).head<3>();
      file << goal_t_WB.x() << " " << goal_t_WB.y() << " " << goal_t_WB.z() << " 255 0 0\n";
    }
    file.close();
  }
  return 0;
}



void SupereightNode::readConfig(const ros::NodeHandle& nh_private) {
  supereight_config_ = read_supereight_config(nh_private);
  ROS_INFO_STREAM(supereight_config_);

  node_config_ = read_supereight_node_config(nh_private);
  print_supereight_node_config(node_config_);
};



void SupereightNode::setupRos() {
  // Initialize the constant messages
  map_dim_msg_ = mapDimMsg();

  // Setup the Habitat-Sim to supereight translation if we're not doing an experiment. This has to
  // happen before pose_sub_ is initialized otherwise the same topic might be subscribed to twice.
  if (node_config_.experiment_type == "habitat") {
    static_tf_broadcaster_.sendTransform(T_WWh_Msg());
  }

  // Pose subscriber
  if (!node_config_.enable_tracking) {
    if (node_config_.pose_topic_type == "geometry_msgs::PoseStamped") {
      pose_sub_ = nh_.subscribe("/pose", node_config_.pose_buffer_size,
          &SupereightNode::poseStampedCallback, this);

    } else if (node_config_.pose_topic_type == "geometry_msgs::TransformStamped") {
      pose_sub_ = nh_.subscribe("/pose", node_config_.pose_buffer_size,
          &SupereightNode::transformStampedCallback, this);

    } else {
      ROS_FATAL("Invalid pose topic type %s", node_config_.pose_topic_type.c_str());
      ROS_FATAL("Expected geometry_msgs::PoseStamped or geometry_msgs::TransformStamped");
      abort();
    }
  }
  // Depth subscriber
  depth_sub_ = nh_.subscribe("/camera/depth_image",
      node_config_.depth_buffer_size, &SupereightNode::depthCallback, this);
  // RGB subscriber
  if (node_config_.enable_rgb) {
    rgb_sub_ = nh_.subscribe("/camera/rgb_image", node_config_.rgb_buffer_size,
        &SupereightNode::RGBCallback, this);
    if (!node_config_.run_segmentation && node_config_.enable_objects) {
      class_sub_ = nh_.subscribe("/camera/class", node_config_.rgb_buffer_size,
          &SupereightNode::SemClassCallback, this);
      instance_sub_ = nh_.subscribe("/camera/instance", node_config_.rgb_buffer_size,
          &SupereightNode::SemInstanceCallback, this);
    }
  }

  // Publishers
  supereight_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/pose",
      node_config_.pose_buffer_size);
  if (node_config_.experiment_type == "gazebo") {
    path_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/supereight/path", 5);
  } else {
    path_pub_ = nh_.advertise<nav_msgs::Path>("/supereight/path", 5);
  }
  static_tf_broadcaster_.sendTransform(T_MW_Msg());
  static_tf_broadcaster_.sendTransform(T_BC_Msg());
  // Render publishers
  if (node_config_.enable_rendering) {
    depth_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/depth_render", 30);
    if (node_config_.enable_rgb) {
      rgba_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/rgba_render", 30);
    }
    if (node_config_.enable_tracking) {
      track_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/track_render",30);
    }
    volume_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render",30);
    volume_render_color_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_color",30);
    volume_render_scale_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_scale",30);
    volume_render_min_scale_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_min_scale",30);
    class_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/class_render",30);
    instance_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/instance_render",30);
    raycast_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/raycast_render",30);
    entropy_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/entropy_render", 30);
    entropy_depth_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/entropy_depth_render", 30);
  }

  // Visualization publishers
  map_dim_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/dim", 1, true);
  map_dim_pub_.publish(map_dim_msg_);
  map_free_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/free", 10);
  map_occupied_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/occupied", 10);
  map_unknown_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/unknown", 10);
  map_mesh_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/mesh", 10);
  map_object_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/objects", 10);
  map_object_mesh_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/object_meshes", 10);
  map_object_aabb_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/object_aabbs", 10);
  map_frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/frontiers", 10);
  map_candidate_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/candidate_views", 100);
  map_candidate_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/candidate_paths", 2);
  map_rejected_candidate_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/rejected_candidate_views", 2);
  map_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/goal", 2);
  mav_sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/mav/sphere", 2);
  pose_history_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/pose_history", 2);
}



void SupereightNode::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
  const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
  depth_buffer_.push_back(depth_msg);
  ROS_DEBUG("Depth image buffer: %lu/%lu", depth_buffer_.size(), depth_buffer_.capacity());
}



void SupereightNode::RGBCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
  const std::lock_guard<std::mutex> rgb_lock (rgb_buffer_mutex_);
  rgb_buffer_.push_back(rgb_msg);
  ROS_DEBUG("RGB image buffer:   %lu/%lu", rgb_buffer_.size(), rgb_buffer_.capacity());
}



void SupereightNode::poseStampedCallback(
    const geometry_msgs::PoseStamped::ConstPtr& T_WB_msg) {

  // Convert the message to an Eigen matrix.
  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(T_WB_msg->pose.orientation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::pointMsgToEigen(T_WB_msg->pose.position, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;

  if (node_config_.center_at_first_position) {
    if (std::isnan(init_t_WB_.x())) {
      // This is the first pose.
      init_t_WB_ = T_WB.topRightCorner<3, 1>().cast<float>();
    }
    // Subtract the initial position.
    T_WB.topRightCorner<3, 1>() -= init_t_WB_.cast<double>();
  }

  // Call the generic pose callback.
  poseCallback(T_WB, T_WB_msg->header);
}



void SupereightNode::transformStampedCallback(
    const geometry_msgs::TransformStamped::ConstPtr& T_WB_msg) {

  // Convert the message to an Eigen matrix.
  Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_WB;
  tf::quaternionMsgToEigen(T_WB_msg->transform.rotation, q_WB);
  T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
  Eigen::Vector3d t_WB;
  tf::vectorMsgToEigen(T_WB_msg->transform.translation, t_WB);
  T_WB.topRightCorner<3, 1>() = t_WB;

  if (node_config_.center_at_first_position) {
    if (std::isnan(init_t_WB_.x())) {
      // This is the first pose.
      init_t_WB_ = T_WB.topRightCorner<3, 1>().cast<float>();
    }
    // Subtract the initial position.
    T_WB.topRightCorner<3, 1>() -= init_t_WB_.cast<double>();
  }

  // Call the generic pose callback.
  poseCallback(T_WB, T_WB_msg->header);
}



void SupereightNode::poseCallback(const Eigen::Matrix4d&  T_WB,
                                  const std_msgs::Header& header) {

  // Convert body pose to camera pose.
  const Eigen::Matrix4d T_WC = T_WB * supereight_config_.T_BC.cast<double>();

  // Create a ROS message from T_WC.
  geometry_msgs::TransformStamped T_WC_msg;
  T_WC_msg.header = header;
  T_WC_msg.header.frame_id = world_frame_id_;
  const Eigen::Quaterniond q_WC (T_WC.topLeftCorner<3, 3>());
  tf::quaternionEigenToMsg(q_WC, T_WC_msg.transform.rotation);
  const Eigen::Vector3d t_WC = T_WC.topRightCorner<3, 1>();
  tf::vectorEigenToMsg(t_WC, T_WC_msg.transform.translation);

  {
    // Put it into the buffer.
    const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
    pose_buffer_.push_back(T_WC_msg);
    ROS_DEBUG("Pose buffer:        %lu/%lu", pose_buffer_.size(), pose_buffer_.capacity());
  }

  // Update the Body-World transform.
  geometry_msgs::TransformStamped T_WB_msg;
  T_WB_msg.header = header;
  T_WB_msg.header.frame_id = world_frame_id_;
  T_WB_msg.child_frame_id = body_frame_id_;
  const Eigen::Quaterniond q_WB (T_WB.topLeftCorner<3, 3>());
  tf::quaternionEigenToMsg(q_WB, T_WB_msg.transform.rotation);
  const Eigen::Vector3d t_WB = T_WB.topRightCorner<3, 1>();
  tf::vectorEigenToMsg(t_WB, T_WB_msg.transform.translation);
  pose_tf_broadcaster_.sendTransform(T_WB_msg);

  if (node_config_.visualization_rate > 0) {
    visualizeMAV();
  }
}



void SupereightNode::SemClassCallback(const sensor_msgs::ImageConstPtr& class_msg) {
  const std::lock_guard<std::mutex> class_lock (class_buffer_mutex_);
  class_buffer_.push_back(class_msg);
  ROS_DEBUG("Class image buffer: %lu/%lu", class_buffer_.size(), class_buffer_.capacity());
}



void SupereightNode::SemInstanceCallback(const sensor_msgs::ImageConstPtr& instance_msg) {
  const std::lock_guard<std::mutex> instance_lock (instance_buffer_mutex_);
  instance_buffer_.push_back(instance_msg);
  ROS_DEBUG("Inst. image buffer: %lu/%lu", instance_buffer_.size(), instance_buffer_.capacity());
}



geometry_msgs::TransformStamped SupereightNode::T_MW_Msg() const {
  // Transform from world frame to map frame. ROS probably uses a different
  // convention than us?
  static geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = map_frame_id_;
  tf.child_frame_id = world_frame_id_;
  tf::vectorEigenToMsg(t_MW_.cast<double>(), tf.transform.translation);
  tf.transform.rotation = make_quaternion();
  return tf;
}



geometry_msgs::TransformStamped SupereightNode::T_BC_Msg() const {
  // Transform from camera frame to body frame. ROS probably uses a different
  // convention than us?
  static geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = body_frame_id_;
  tf.child_frame_id = camera_frame_id_;
  const Eigen::Matrix4d T_BC = supereight_config_.T_BC.cast<double>();
  const Eigen::Quaterniond q_BC (T_BC.topLeftCorner<3, 3>());
  tf::quaternionEigenToMsg(q_BC, tf.transform.rotation);
  const Eigen::Vector3d t_BC = T_BC.topRightCorner<3, 1>();
  tf::vectorEigenToMsg(t_BC, tf.transform.translation);
  return tf;
}



geometry_msgs::TransformStamped SupereightNode::T_WWh_Msg() const {
  static geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = world_frame_id_;
  tf.child_frame_id = "habitat_world";
  // Wait for a pose from Habitat-Sim and use its translation
  // Don't use the rotation because it messes stuff up
  ROS_INFO("Waiting for pose from Habitat-Sim");
  if (node_config_.pose_topic_type == "geometry_msgs::PoseStamped") {
    geometry_msgs::PoseStampedConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");
    tf.transform.translation.x = msg->pose.position.x;
    tf.transform.translation.y = msg->pose.position.y;
    tf.transform.translation.z = msg->pose.position.z;
    tf.transform.rotation = make_quaternion();

  } else if (node_config_.pose_topic_type == "geometry_msgs::TransformStamped") {
    geometry_msgs::TransformStampedConstPtr msg = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/pose");
    tf.transform.translation.x = msg->transform.translation.x;
    tf.transform.translation.y = msg->transform.translation.y;
    tf.transform.translation.z = msg->transform.translation.z;
    tf.transform.rotation = make_quaternion();


  } else {
    ROS_FATAL("Invalid pose topic type %s", node_config_.pose_topic_type.c_str());
    ROS_FATAL("Expected geometry_msgs::PoseStamped or geometry_msgs::TransformStamped");
    abort();
  }
  return tf;
}



visualization_msgs::Marker SupereightNode::mapDimMsg() const {
  static visualization_msgs::Marker m;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = map_frame_id_;
  m.ns = "map_dim";
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position = make_point(supereight_config_.map_dim.x() / 2.0f);
  m.pose.orientation = make_quaternion();
  m.scale = make_vector3(supereight_config_.map_dim.x());
  m.color = make_color(1.0f, 1.0f, 1.0f, 0.1f);
  m.lifetime = ros::Duration(0.0);
  m.frame_locked = true;
  return m;
}



void SupereightNode::printFrameTimes() const {
  double total_time = 0.0;
  total_time += times_matching_.back();
  total_time += times_preprocessing_.back();
  total_time += times_tracking_.back();
  total_time += times_integration_.back();
  total_time += times_object_integration_.back();
  total_time += times_rendering_.back();
  total_time += times_visualization_.back();
  ROS_INFO("%-25s %.5f s", "Matching",           times_matching_.back());
  ROS_INFO("%-25s %.5f s", "Preprocessing",      times_preprocessing_.back());
  ROS_INFO("%-25s %.5f s", "Tracking",           times_tracking_.back());
  ROS_INFO("%-25s %.5f s", "Integration",        times_integration_.back());
  ROS_INFO("%-25s %.5f s", "Object integration", times_object_integration_.back());
  ROS_INFO("%-25s %.5f s", "Rendering",          times_rendering_.back());
  ROS_INFO("%-25s %.5f s", "Visualization",      times_visualization_.back());
  ROS_INFO("Frame total               %.5f s", total_time);
}



void SupereightNode::runNetwork(const Eigen::Matrix4f&            T_WC,
                                const sensor_msgs::ImageConstPtr& depth_image,
                                const sensor_msgs::ImageConstPtr& color_image) {
  // runNetwork() should only be run by a single thread at a time so that matchAndFuse() can fall
  // back to calling fuse(). Return if the lock can't be acquired (another thread is already
  // running).
  std::unique_lock<std::mutex> network_lock (network_mutex_, std::defer_lock_t());
  if (!network_lock.try_lock()) {
    return;
  }

  std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();

  cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(color_image, "rgb8");
  se::SegmentationResult segmentation (rgb_ptr->image.cols, rgb_ptr->image.rows);
#ifdef SE_WITH_MASKRCNN
  const std::vector<mr::Detection> detections = network_.infer(rgb_ptr->image, false);
  //cv::imwrite("/home/srl/frame_" + std::to_string(frame_) + ".png",
  //    mr::visualize_detections(detections, rgb_ptr->image));
  // Convert the object detections to a SegmentationResult.
  for (const auto& d : detections) {
    segmentation.object_instances.emplace_back(se::instance_new, d.mask,
        se::DetectionConfidence(d.class_id, d.confidence));
  }
#endif // SE_WITH_MASKRCNN

  std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
  times_network_.push_back(std::chrono::duration<double>(end_time - start_time).count());
  ROS_INFO("%-25s %.5f s", "Network", times_network_.back());

  fuse(T_WC, depth_image, color_image, segmentation);
}
}  // namespace se

