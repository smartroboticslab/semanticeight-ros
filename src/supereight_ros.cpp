// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros.hpp"

#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <thread>

#include <lodepng.h>

#include <eigen_conversions/eigen_msg.h>

#include "se/config.h"

#include "supereight_ros/utilities.hpp"
#include "se/voxel_implementations.hpp"



namespace se {

SupereightNode::SupereightNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      sensor_({1, 1, false, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}),
      frame_(0),
      world_frame_id_("world"),
      map_frame_id_("map"),
      body_frame_id_("body"),
      camera_frame_id_("camera") {

  readConfig(nh_private);

  t_MW_ = supereight_config_.t_MW_factor.cwiseProduct(supereight_config_.map_dim);
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
  }

  // Initialize the sensor.
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
  sensor_ = SensorImpl({node_config_.input_res.x(), node_config_.input_res.y(),
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

  // Initialize the timings.
  timings_.resize(8);
  timing_labels_ = {"Message preprocessing",
                    "Preprocessing",
                    "Tracking",
                    "Integration",
                    "Raycasting",
                    "Rendering",
                    "Visualization"};

  // Allocate message circular buffers.
  if (node_config_.enable_tracking) {
    pose_buffer_.set_capacity(0);
  } else {
    pose_buffer_.set_capacity(node_config_.pose_buffer_size);
  }
  depth_buffer_.set_capacity(node_config_.depth_buffer_size);
  if (node_config_.enable_rgb) {
    rgb_buffer_.set_capacity(node_config_.rgb_buffer_size);
  } else {
    rgb_buffer_.set_capacity(0);
  }

  setupRos();

  ROS_INFO("Initialization finished");
}



void SupereightNode::runPipelineOnce() {
  // runPipelineOnce() should only be run by a single thread at a time. Return
  // if the lock can't be acquired (another thread is already running).
  std::unique_lock<std::mutex> fusion_lock (fusion_mutex_, std::defer_lock_t());
  if (!fusion_lock.try_lock()) {
    return;
  }

  timings_[0] = std::chrono::steady_clock::now();



  // Message association
  // Depth
  sensor_msgs::ImageConstPtr current_depth_msg;
  double depth_timestamp;
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    if (depth_buffer_.empty()) {
      ROS_DEBUG("Aborted fusion: depth buffer empty");
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
      ROS_DEBUG("Aborted fusion: RGB buffer empty");
      return;
    } else {
      const bool found = get_closest_image(rgb_buffer_, depth_timestamp,
          node_config_.max_timestamp_diff, current_rgb_msg);
      if (!found) {
        ROS_DEBUG("Aborted fusion: could not find matching RGB");
        return;
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
      ROS_DEBUG("Aborted fusion: pose buffer empty");
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
        ROS_DEBUG("Aborted fusion: query smaller than all poses");
        return;
      } else if (result == InterpResult::query_greater) {
        // Remove the first poses, they will never be matched to depth images.
        pose_buffer_.erase_begin(pose_buffer_.size() - 1);
        ROS_DEBUG("Aborted fusion: query greater than all poses");
        return;
      }

      // Interpolate to associate a pose to the depth image.
      external_T_WC = interpolate_pose(prev_pose, next_pose, depth_timestamp);
    }
  }

  // The currect depth image is going to be integrated, remove it from the
  // buffer to avoid integrating it again.
  { // Block to reduce the scope of depth_lock.
    const std::lock_guard<std::mutex> depth_lock (depth_buffer_mutex_);
    depth_buffer_.pop_front();
  }
  // Copy the depth and RGB images into the buffers used by supereight.
  to_supereight_depth(*current_depth_msg, sensor_.far_plane, input_depth_.get());
  if (node_config_.enable_rgb) {
    to_supereight_RGB(*current_rgb_msg, input_rgba_.get());
  }
  timings_[1] = std::chrono::steady_clock::now();



  // Preprocessing
  pipeline_->preprocessDepth(input_depth_.get(), node_config_.input_res,
      supereight_config_.bilateral_filter);
  if (node_config_.enable_rgb) {
    pipeline_->preprocessColor(input_rgba_.get(), node_config_.input_res);
  }
  timings_[2] = std::chrono::steady_clock::now();



  // Tracking
  bool tracked = false;
  if (node_config_.enable_tracking) {
    if (frame_ % supereight_config_.tracking_rate == 0) {
      tracked = pipeline_->track(sensor_, supereight_config_.icp_threshold);
    } else {
      tracked = false;
    }
  } else {
    pipeline_->setT_WC(external_T_WC);
    tracked = true;
  }

  // Publish pose estimated/received by supereight.
  const Eigen::Matrix4f se_T_WB = pipeline_->T_WC() * T_CB_;
  const Eigen::Vector3d se_t_WB = se_T_WB.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond se_q_WB (se_T_WB.block<3, 3>(0, 0).cast<double>());
  geometry_msgs::PoseStamped supereight_pose;
  supereight_pose.header = current_depth_msg->header;
  supereight_pose.header.frame_id = world_frame_id_;
  tf::pointEigenToMsg(se_t_WB, supereight_pose.pose.position);
  tf::quaternionEigenToMsg(se_q_WB, supereight_pose.pose.orientation);
  supereight_pose_pub_.publish(supereight_pose);
  timings_[3] = std::chrono::steady_clock::now();



  // Integration
  // Integrate only if tracking was successful or it is one of the first 4
  // frames.
  bool integrated = false;
  if ((tracked && (frame_ % supereight_config_.integration_rate == 0)) || frame_ <= 3) {
    integrated = pipeline_->integrate(sensor_, frame_);
  } else {
    integrated = false;
  }
  timings_[4] = std::chrono::steady_clock::now();



  // Raycasting
  bool raycasted = false;
  if ((node_config_.enable_tracking || node_config_.enable_rendering) && frame_ > 2) {
    raycasted = pipeline_->raycast(sensor_);
  }
  timings_[5] = std::chrono::steady_clock::now();



  // Rendering
  if (node_config_.enable_rendering) {
    // Depth
    pipeline_->renderDepth(depth_render_.get(), image_res_, sensor_);
    const sensor_msgs::Image depth_render_msg = RGBA_to_msg(depth_render_.get(),
        image_res_, current_depth_msg->header);
    depth_render_pub_.publish(depth_render_msg);

    // RGB
    if (node_config_.enable_rgb) {
      pipeline_->renderRGBA(rgba_render_.get(), image_res_);
      const sensor_msgs::Image rgba_render_msg = RGBA_to_msg(rgba_render_.get(),
          image_res_, current_depth_msg->header);
      rgba_render_pub_.publish(rgba_render_msg);
    }

    // Track
    if (node_config_.enable_tracking) {
      pipeline_->renderTrack(track_render_.get(), image_res_);
      const sensor_msgs::Image track_render_msg = RGBA_to_msg(track_render_.get(),
          image_res_, current_depth_msg->header);
      track_render_pub_.publish(track_render_msg);
    }

    // Volume
    if (frame_ % supereight_config_.rendering_rate == 0) {
      pipeline_->renderVolume(volume_render_.get(), image_res_, sensor_);
      const sensor_msgs::Image volume_render_msg = RGBA_to_msg(volume_render_.get(),
          image_res_, current_depth_msg->header);
      volume_render_pub_.publish(volume_render_msg);
    }
  }
  timings_[6] = std::chrono::steady_clock::now();



  // Visualization
  if (node_config_.visualization_rate > 0 && (frame_ % node_config_.visualization_rate == 0)) {
    visualizeWholeMap();
  }
  timings_[7] = std::chrono::steady_clock::now();



  ROS_INFO("-----------------------------------------");
  ROS_INFO("Frame %d", frame_);
  ROS_INFO("Tracked: %d   Integrated: %d   Raycasted: %d",
      tracked, integrated, raycasted);
  print_timings(timings_, timing_labels_);

  frame_++;
}



void SupereightNode::saveMap() {
  if (!supereight_config_.output_mesh_file.empty()) {
    pipeline_->dumpMesh(supereight_config_.output_mesh_file.c_str());
    ROS_INFO("Map saved in %s\n", supereight_config_.output_mesh_file.c_str());
  }
}



void SupereightNode::readConfig(const ros::NodeHandle& nh_private) {
  supereight_config_ = read_supereight_config(nh_private);
  ROS_INFO_STREAM(supereight_config_);

  node_config_ = read_supereight_node_config(nh_private);
  print_supereight_node_config(node_config_);
};



void SupereightNode::setupRos() {
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
  }

  // Publishers
  supereight_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/pose",
      node_config_.pose_buffer_size);
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
  }

  // Visualization publishers
  map_dim_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/dim", 1, true);
  map_dim_pub_.publish(mapDimMsg());
  map_free_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/free", 1);
  map_occupied_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/occupied", 1);
  map_unknown_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/unknown", 1);
}



geometry_msgs::TransformStamped SupereightNode::T_MW_Msg() {
  // Transform from world frame to map frame. ROS probably uses a different
  // convention than us?
  geometry_msgs::TransformStamped tf;
  tf.header.seq = 0;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = map_frame_id_;
  tf.child_frame_id = world_frame_id_;
  tf::vectorEigenToMsg(t_MW_.cast<double>(), tf.transform.translation);
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;
  return tf;
}



geometry_msgs::TransformStamped SupereightNode::T_BC_Msg() {
  // Transform from camera frame to body frame. ROS probably uses a different
  // convention than us?
  geometry_msgs::TransformStamped tf;
  tf.header.seq = 0;
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



visualization_msgs::Marker SupereightNode::mapDimMsg() {
  visualization_msgs::Marker m;
  m.header.seq = 0;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = map_frame_id_;
  m.ns = "map_dim";
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = supereight_config_.map_dim.x() / 2.0f;
  m.pose.position.y = supereight_config_.map_dim.x() / 2.0f;
  m.pose.position.z = supereight_config_.map_dim.x() / 2.0f;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = supereight_config_.map_dim.x();
  m.scale.y = supereight_config_.map_dim.x();
  m.scale.z = supereight_config_.map_dim.x();
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 0.1;
  m.lifetime = ros::Duration(0.0);
  m.frame_locked = true;
  return m;
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

  // Put it into the buffer.
  const std::lock_guard<std::mutex> pose_lock (pose_buffer_mutex_);
  pose_buffer_.push_back(T_WC_msg);
  ROS_DEBUG("Pose buffer:        %lu/%lu",
      pose_buffer_.size(), pose_buffer_.capacity());
}



void SupereightNode::visualizeWholeMap() {
  // Initialize the message header
  static uint32_t seq = 0;
  std_msgs::Header header;
  header.seq = seq++;
  header.stamp = ros::Time::now();
  header.frame_id = map_frame_id_;
  // Iterate over the octree, creating a different CUBE_LIST marker for each
  // volume size and state.
  std::map<int, visualization_msgs::Marker> markers_free;
  std::map<int, visualization_msgs::Marker> markers_occupied;
  std::map<int, visualization_msgs::Marker> markers_unknown;
  for (const auto& volume : *(pipeline_->getMap())) {
    // Select a marker list and color depending on the volume state.
    std::map<int, visualization_msgs::Marker>* markers = nullptr;
    std::string ns;
    std_msgs::ColorRGBA volume_color;
    if (is_free(volume)) {
      markers = &markers_free;
      ns = "map_free";
      volume_color.r = color_free_.x();
      volume_color.g = color_free_.y();
      volume_color.b = color_free_.z();
      volume_color.a = color_free_.w();
    } else if (is_occupied(volume)) {
      markers = &markers_occupied;
      ns = "map_occupied";
      volume_color.r = color_occupied_.x();
      volume_color.g = color_occupied_.y();
      volume_color.b = color_occupied_.z();
      volume_color.a = color_occupied_.w();
    } else {
      markers = &markers_unknown;
      ns = "map_unknown";
      volume_color.r = color_unknown_.x();
      volume_color.g = color_unknown_.y();
      volume_color.b = color_unknown_.z();
      volume_color.a = color_unknown_.w();
    }
    const int size = volume.size;
    if (markers->count(size) == 0) {
      // Initialize the Marker message for this cube size.
      (*markers)[size] = visualization_msgs::Marker();
      (*markers)[size].header = header;
      (*markers)[size].ns = ns;
      (*markers)[size].id = size;
      (*markers)[size].type = visualization_msgs::Marker::CUBE_LIST;
      (*markers)[size].action = visualization_msgs::Marker::ADD;
      (*markers)[size].pose.orientation.x = 0.0;
      (*markers)[size].pose.orientation.y = 0.0;
      (*markers)[size].pose.orientation.z = 0.0;
      (*markers)[size].pose.orientation.w = 1.0;
      (*markers)[size].scale.x = volume.dim;
      (*markers)[size].scale.y = volume.dim;
      (*markers)[size].scale.z = volume.dim;
      (*markers)[size].color = volume_color;
      (*markers)[size].lifetime = ros::Duration(0.0);
      (*markers)[size].frame_locked = true;
    }
    // Append the current volume.
    geometry_msgs::Point p;
    p.x = volume.centre_M.x();
    p.y = volume.centre_M.y();
    p.z = volume.centre_M.z();
    (*markers)[size].points.push_back(p);
  }
  // Publish all markers.
  for (const auto& marker : markers_free) {
    map_free_pub_.publish(marker.second);
  }
  for (const auto& marker : markers_occupied) {
    map_occupied_pub_.publish(marker.second);
  }
  for (const auto& marker : markers_unknown) {
    map_unknown_pub_.publish(marker.second);
  }
}



bool SupereightNode::is_free(const se::Volume<VoxelImpl::VoxelType>& volume) const {
  constexpr bool is_tsdf = VoxelImpl::invert_normals;
  return (is_tsdf && volume.data.x > 0.0f) || (!is_tsdf && volume.data.x < 0.0f);
}



bool SupereightNode::is_occupied(const se::Volume<VoxelImpl::VoxelType>& volume) const {
  constexpr bool is_tsdf = VoxelImpl::invert_normals;
  return (is_tsdf && volume.data.x < 0.0f) || (!is_tsdf && volume.data.x > 0.0f);
}

}  // namespace se

