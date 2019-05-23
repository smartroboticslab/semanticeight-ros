//
// Created by anna on 12/04/19.
//

#ifndef SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP
#define SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP

#include <iostream>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <se/lodepng.h>
//#include <se/default_parameters.h>
#include <supereight_ros/supereight_ros.hpp>
namespace se {
/**
* for ROS param function
* default value list, same as in config.yaml
*/

#define DEFAULT_ITERATION_COUNT 3
static const int default_iterations[DEFAULT_ITERATION_COUNT] = {10, 5, 4};
const int default_compute_size_ratio = 2;
const int default_tracking_rate = 1;
const int default_integration_rate = 3;
const int default_rendering_rate = 4;
const Eigen::Vector3i default_volume_resolution(256, 256, 256);
const Eigen::Vector3f default_volume_size(6.f, 6.f, 6.f);
const Eigen::Vector3f default_initial_pos_factor(0.5f, 0.5f, 0.0f);
const std::string default_dump_volume_file = "";
const std::string default_input_file = "";
const std::string default_log_file = "";
const std::string default_groundtruth_file = "";
const Eigen::Matrix4f default_gt_transform = Eigen::Matrix4f::Identity();
const float default_mu = 0.1f;
const int default_fps = 0;
const bool default_blocking_read = false;
const float default_icp_threshold = 1e-6;
const bool default_no_gui = false;
const bool default_render_volume_fullsize = false;
const bool default_bilateral_filter = false;
const int default_coloured_voxels = false;
const int default_multi_resolution = false;
const bool default_bayesian = false;

std::string sep = "\n----------------------------------------\n";

template<typename T>
void SupereightNode<T>::setupRos() {
  // Subscriber
  image_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw",
                             100,
                             &SupereightNode::imageCallback,
                             this);
  pose_sub_ = nh_.subscribe("/vicon/d435/d435", 1000, &SupereightNode::poseCallback, this);
  image_pose_sub_ =
      nh_.subscribe("/supereight/image_pose", 100, &SupereightNode::fusionCallback, this);
  cam_info_sub_ = nh_.subscribe("/camera/camera_info", 2, &SupereightNode::camInfoCallback, this);


  // Publisher
  image_pose_pub_ = nh_.advertise<supereight_ros::ImagePose>("/supereight/image_pose", 1000);
  supereight_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/pose", 1000);
  gt_tf_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/gt_tf_pose", 1000);

  // TODO ifdef
#ifdef WITH_RENDERING
  depth_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/depth_render", 30);
  volume_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render",30);
  track_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/track_render",30);
#endif
  // Visualization
  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_based_marker", 1);
  block_based_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_based_marker", 1);
  block_based_marker_array_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("block_based_marker_array", 1);
  std::string ns = ros::this_node::getName();
  bool read_from_launch = ros::param::get(ns + "/enable_icp_tracking_", enable_icp_tracking_);
  ros::param::get(ns + "/use_tf_transforms_", use_tf_transforms_);
}

// Read config.yaml into Configuration class
template<typename T>
void SupereightNode<T>::setSupereightConfig(const ros::NodeHandle &nh_private) {
  ROS_INFO("set Supereight config");
  nh_private.param<int>("compute_size_ratio",
                        supereight_config_.compute_size_ratio,
                        default_compute_size_ratio);

  nh_private.param<int>("tracking_rate", supereight_config_.tracking_rate, default_tracking_rate);

  nh_private.param<int>("integration_rate",
                        supereight_config_.integration_rate,
                        default_integration_rate);

  nh_private.param<int>("integration_rate",
                        supereight_config_.rendering_rate,
                        default_rendering_rate);

  std::vector<int> volume_resolution_vector;
  if (nh_private.getParam("volume_resolution", volume_resolution_vector)) {
    for (unsigned int i = 0; i < volume_resolution_vector.size(); i++) {
      supereight_config_.volume_resolution[i] = volume_resolution_vector[i];
    }
  } else {
    supereight_config_.volume_resolution = default_volume_resolution;
  }

  std::vector<float> volume_size_vector;
  if (nh_private.getParam("volume_size", volume_size_vector)) {
    for (unsigned int i = 0; i < volume_size_vector.size(); i++) {
      supereight_config_.volume_size[i] = volume_size_vector[i];
    }
  } else {
    supereight_config_.volume_size = default_volume_size;
  }

  std::vector<float> initial_pos_factor_vector;
  if (nh_private.getParam("initial_pos_factor", initial_pos_factor_vector)) {
    for (unsigned int i = 0; i < initial_pos_factor_vector.size(); i++) {
      supereight_config_.initial_pos_factor[i] = initial_pos_factor_vector[i];
    }
  } else {
    supereight_config_.initial_pos_factor = default_initial_pos_factor;
  }

  std::vector<int> pyramid;
  if (!nh_private.getParam("pyramid", pyramid)) {
    supereight_config_.pyramid.clear();
    for (int i = 0; i < DEFAULT_ITERATION_COUNT; i++) {
      supereight_config_.pyramid.push_back(default_iterations[i]);
    }
  }

  nh_private.param<std::string>("dump_volume_file",
                                supereight_config_.dump_volume_file,
                                default_dump_volume_file);

  nh_private.param<std::string>("input_file", supereight_config_.input_file, default_input_file);

  nh_private.param<std::string>("log_file", supereight_config_.log_file, default_log_file);

  nh_private.param<std::string>("groundtruth_file",
                                supereight_config_.groundtruth_file,
                                default_groundtruth_file);

  std::vector<float> gt_transform_vector;
  nh_private.getParam("gt_transform", gt_transform_vector);
  if (nh_private.getParam("gt_transform", gt_transform_vector)) {
    for (unsigned int i = 0; i < std::sqrt(gt_transform_vector.size()); i++) {
      for (unsigned int j = 0; j < std::sqrt(gt_transform_vector.size()); j++) {
        supereight_config_.gt_transform(i, j) = gt_transform_vector[i * 4 + j];
      }
    }
  } else {
    supereight_config_.gt_transform = default_gt_transform;
  }

  std::vector<float> camera_vector;
  if (!nh_private.getParam("camera", camera_vector)) {
    ros::shutdown();
  }
  for (unsigned int i = 0; i < camera_vector.size(); i++) {
    supereight_config_.camera[i] = camera_vector[i];
  }
  supereight_config_.camera_overrided = true;

  /**
   * The TSDF truncation bound. Values of the TSDF are assumed to be in the
   * interval ±mu. See Section 3.3 of \cite NewcombeISMAR2011 for more
   * details.
   *  <br>\em Default: 0.1
   */
  nh_private.param<float>("mu", supereight_config_.mu, default_mu);

  nh_private.param<int>("fps", supereight_config_.fps, default_fps);

  nh_private.param<bool>("blocking_read", supereight_config_.blocking_read, default_blocking_read);

  nh_private.param<float>("icp_threshold", supereight_config_.icp_threshold, default_icp_threshold);

  nh_private.param<bool>("no_gui", supereight_config_.no_gui, default_no_gui);

  nh_private.param<bool>("render_volume_fullsize",
                         supereight_config_.render_volume_fullsize,
                         default_render_volume_fullsize);

  nh_private.param<bool>("bilateral_filter",
                         supereight_config_.bilateral_filter,
                         default_bilateral_filter);

  nh_private.param<bool>("coloured_voxels",
                         supereight_config_.coloured_voxels,
                         default_coloured_voxels);

  nh_private.param<bool>("multi_resolution",
                         supereight_config_.multi_resolution,
                         default_multi_resolution);

  nh_private.param<bool>("bayesian", supereight_config_.bayesian, default_bayesian);

  std::vector<int> image_size_vector;
  if (nh_private.getParam("input_size", image_size_vector)) {
    for (unsigned int i = 0; i < image_size_vector.size(); i++) {
      image_size_[i] = image_size_vector[i];
    }
  }

  for (int i = 0; i < volume_size_vector.size(); i++) {
    init_position_octree_[i] = volume_size_vector[i] * initial_pos_factor_vector[i];
  }


};

// print configuration to terminal
template<typename T>
void SupereightNode<T>::printSupereightConfig(const Configuration &config) {
  std::cout << "compute_size_ratio = " << config.compute_size_ratio << std::endl;
  std::cout << "tracking_rate = " << config.tracking_rate << std::endl;
  std::cout << "integration_rate = " << config.integration_rate << std::endl;
  std::cout << "rendering_rate = " << config.rendering_rate << std::endl;
  std::cout << "volume_resolution = \n" << config.volume_resolution << std::endl;
  std::cout << "volume_size = \n" << config.volume_size << std::endl;
  std::cout << "initial_pos_factor = \n" << config.initial_pos_factor << std::endl;
  std::cout << "pyramid = \n" << config.pyramid[0] << " " << config.pyramid[1] << " "
            << config.pyramid[2] << std::endl;
  std::cout << "dump_volume_file = " << config.dump_volume_file << std::endl;
  std::cout << "input_file = " << config.input_file << std::endl;
  std::cout << "log_file = " << config.log_file << std::endl;
  std::cout << "groundtruth_file = " << config.groundtruth_file << std::endl;
  std::cout << "gt_transform = \n" << config.gt_transform << std::endl;
  std::cout << "camera = \n" << config.camera << std::endl;
  std::cout << "camera_overrided = " << config.camera_overrided << std::endl;
  std::cout << "mu = " << config.mu << std::endl;
  std::cout << "fps = " << config.fps << std::endl;
  std::cout << "blocking_read = " << config.blocking_read << std::endl;
  std::cout << "icp_threshold = " << config.icp_threshold << std::endl;
  std::cout << "no_gui = " << config.no_gui << std::endl;
  std::cout << "render_volume_fullsize = " << config.render_volume_fullsize << std::endl;
  std::cout << "bilateral_filter = " << config.bilateral_filter << std::endl;
  std::cout << "coloured_voxels = " << config.coloured_voxels << std::endl;
  std::cout << "multi_resolution = " << config.multi_resolution << std::endl;
  std::cout << "bayesian = " << config.bayesian << std::endl;
}

template<typename T>
void SupereightNode<T>::imageCallback(const sensor_msgs::ImageConstPtr &image_msg) {

//   allocate new depth image message
  sensor_msgs::ImagePtr depth_image(new sensor_msgs::Image());
  CreateImageMsg(image_msg, depth_image);

  depth_image->data.resize(depth_image->height * depth_image->step, 0.0f);

  if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    float constant = CamModel.fx() * cam_baseline_; // [px]*[m]
    ConvertDisp2Depth(image_msg, depth_image, constant);
  } else {
    ROS_ERROR("Disparity image has unsupported encoding [%s]", image_msg->encoding.c_str());
    return;
  }

  image_queue_.push_back(*image_msg);
  if (image_queue_.size() == 1) {
    image_time_stamp_ = ros::Time(image_queue_.front().header.stamp).toNSec();
  }
}

template<typename T>
void SupereightNode<T>::camInfoCallback(const sensor_msgs::CameraInfoConstPtr &camInfoIn) {
//  ROS_INFO("cam info callback %i", cam_info_ready_);
  if (cam_info_ready_) return;
  CamInfo = *camInfoIn;
  CamModel.fromCameraInfo(camInfoIn);

  cam_info_ready_ = true;
  ROS_INFO("got camera model..");
//  std::cout << CamModel.cameraInfo() << std::endl;
//  std::cout << CamModel.fx() << "," << CamModel.fy() << std::endl;
}

template<typename T>
void SupereightNode<T>::poseCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_msg) {
//  ROS_INFO("pose call back");
  Eigen::Quaternionf rot_q(pose_msg->transform.rotation.w,
                           pose_msg->transform.rotation.x,
                           pose_msg->transform.rotation.y,
                           pose_msg->transform.rotation.z);
  Eigen::Vector3f translation(pose_msg->transform.translation.x,
                              pose_msg->transform.translation.y,
                              pose_msg->transform.translation.z);

  geometry_msgs::TransformStamped pose_tf_msg_;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  // set Eigen pose from ROS msg
  pose.block<3, 1>(0, 3) = translation;
  pose.block<3, 3>(0, 0) = rot_q.toRotationMatrix();

  // world frame to octree frame rotation transformation
  pose = SwapAxes_octree_world(pose);

  // initialize map from world transformation matrix
  if (!set_world_to_map_tf_) {
    const_translation_ = init_position_octree_;
    const_translation_(0) -= translation(0);
    const_translation_(1) -= translation(2);
    const_translation_(2) -= translation(1);
    // x- axis 90, z-axis 90
    tf_map_from_world_ << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    tf_map_from_world_.block<3, 1>(0, 3) -= init_position_octree_;
    set_world_to_map_tf_ = true;
    std::cout << "world to map tf \n" <<translation << " \n" << tf_map_from_world_ << sep;
  }
  // continuous transformation
  pose.block<3, 1>(0, 3) -= const_translation_;
  pose = tf_map_from_world_ * pose;

  // convert bag to ROS msg
  tf::StampedTransform tf_pose_msg;
  rot_q = Eigen::Quaternionf(pose.block<3, 3>(0, 0));
  translation = pose.block<3, 1>(0, 3);
  tf::vectorEigenToMsg(translation.cast<double>(), pose_tf_msg_.transform.translation);
  tf::quaternionEigenToMsg(rot_q.cast<double>(), pose_tf_msg_.transform.rotation);
  pose_tf_msg_.header = pose_msg->header;
  pose_tf_msg_.header.frame_id = frame_id_;

  // insert to ring buffer
  pose_buffer_.put(pose_tf_msg_);
  while (image_queue_.size() > 0
      && (ros::Time(pose_msg->header.stamp).toNSec() > image_time_stamp_)) {
    supereight_ros::ImagePose image_pose_msg;

    image_pose_msg.image = image_queue_.front();

    geometry_msgs::TransformStamped pre_pose;
    geometry_msgs::TransformStamped post_pose;

    pose_buffer_.get(image_time_stamp_, pre_pose, post_pose);

    image_pose_msg.pre_pose = pre_pose;
    image_pose_msg.post_pose = post_pose;

    // fusion callback
    image_pose_pub_.publish(image_pose_msg);

    image_queue_.pop_front();

    if (image_queue_.size() > 0)
      image_time_stamp_ = ros::Time(image_queue_.front().header.stamp).toNSec();
  }

  //publish the transformed vi_sensor_ground truth
  geometry_msgs::PoseStamped gt_tf_pose_msg;
  gt_tf_pose_msg.header = pose_msg->header;
  gt_tf_pose_msg.header.frame_id = frame_id_;
  gt_tf_pose_msg.pose.position.x = pose_tf_msg_.transform.translation.x+init_position_octree_(0);
  gt_tf_pose_msg.pose.position.y = pose_tf_msg_.transform.translation.y+init_position_octree_(0);
  gt_tf_pose_msg.pose.position.z = pose_tf_msg_.transform.translation.z+init_position_octree_(0);
  gt_tf_pose_msg.pose.orientation = pose_tf_msg_.transform.rotation;
  gt_tf_pose_pub_.publish(gt_tf_pose_msg);
}

template<typename T>
void SupereightNode<T>::fusionCallback(const supereight_ros::ImagePose::ConstPtr &image_pose_msg) {
  bool integrated = false;
  bool raycasted = false;
  bool tracked = false;

  // creates file with poses
//  myfile.open("/home/anna/Data/octree.txt", std::ofstream::app);

  std::chrono::time_point<std::chrono::steady_clock> timings[9];
  ROS_INFO_STREAM("fusion call back");

  timings[0] = std::chrono::steady_clock::now();
  const float *imagepointer = (const float *) image_pose_msg->image.data.data();
  for (int p = 0; p < image_size_.y() * image_size_.x(); p++) {
    if (imagepointer[p] != imagepointer[p]) {
      input_depth_[p] = 0;
    } else {
      input_depth_[p] = static_cast<uint16_t>( 1000 * imagepointer[p]);
//      ROS_INFO("depth %i, %f", input_depth_[p], imagepointer[p]);
    }
  }
//  std::cout << input_depth_[150 * 640 + 320] << std::endl;
//  save image to png
//  lodepng_encode_file("/home/anna/image_file.png",
//                      (unsigned char *) input_depth_,
//                      image_size_.x(),
//                      image_size_.y(),
//                      LCT_GREY,
//                      16);

  timings[1] = std::chrono::steady_clock::now();
  Eigen::Matrix4f gt_pose = interpolatePose(image_pose_msg->pre_pose,
                                            image_pose_msg->post_pose,
                                            ros::Time(image_pose_msg->image.header.stamp).toNSec());
//  myfile<<std::fixed<<gt_pose<<std::endl;
//  myfile.close();

  //-------- supereight access point ----------------
  timings[2] = std::chrono::steady_clock::now();

  pipeline_->preprocessing(input_depth_, image_size_, supereight_config_.bilateral_filter);
  ROS_INFO_STREAM("preprocessed ");
  timings[3] = std::chrono::steady_clock::now();

  Eigen::Vector4f camera = supereight_config_.camera / (supereight_config_.compute_size_ratio);
  if (enable_icp_tracking_) {
    tracked = pipeline_->tracking(camera,
                                  supereight_config_.icp_threshold,
                                  supereight_config_.tracking_rate,
                                  frame_);

    ROS_DEBUG("tracking");
  } else {
    pipeline_->setPose(gt_pose);
    tracked = true;
    ROS_DEBUG("set pose");
  }

  Eigen::Matrix4f tracked_pose = pipeline_->getPose();
  std::cout << "estimated pose by supereight \n" << tracked_pose << sep;
  geometry_msgs::PoseStamped supereight_pose;
  supereight_pose.header = image_pose_msg->image.header;
  supereight_pose.header.frame_id = frame_id_;
  tf::pointEigenToMsg(tracked_pose.block<3,1>(0,3).cast<double>(), supereight_pose.pose.position);
  Eigen::Quaternionf q_rot(tracked_pose.block<3, 3>(0, 0));
  tf::quaternionEigenToMsg(q_rot.cast<double>(), supereight_pose.pose.orientation);

// publish poses estimated using supereight map
  supereight_pose_pub_.publish(supereight_pose);

  timings[4] = std::chrono::steady_clock::now();

  // for visualization
  std::vector<Eigen::Vector3i> occupied_voxels;
  std::vector<Eigen::Vector3i> freed_voxels;
  std::vector<Eigen::Vector3i> updated_blocks;

  // Integrate only if tracking was successful or it is one of the
  // first 4 frames.
  if (tracked || frame_ <= 3) {
    integrated = pipeline_->integration(camera,
                                        supereight_config_.integration_rate,
                                        supereight_config_.mu,
                                        frame_,
                                        &updated_blocks);

  } else {
    integrated = false;
  }

  timings[5] = std::chrono::steady_clock::now();

  pipeline_->raycasting(camera, supereight_config_.mu, frame_);
  timings[6] = std::chrono::steady_clock::now();

  ROS_INFO("integrated %i, tracked %i ", integrated, tracked);
  ROS_INFO_STREAM("occupied_voxels = " << occupied_voxels.size());
  //  ROS_INFO_STREAM( "freed_voxels = " << freed_voxels.size());
  ROS_INFO_STREAM("updated voxels = " << updated_blocks.size());

// ------------ supereight tracking visualization  ---------------------
#ifdef WITH_RENDERING

  pipeline_->renderDepth((unsigned char *) depth_render_, pipeline_->getComputationResolution());
  pipeline_->renderTrack((unsigned char *) track_render_, pipeline_->getComputationResolution());
  pipeline_->renderVolume((unsigned char *) volume_render_,
                          pipeline_->getComputationResolution(),
                          frame_,
                          supereight_config_.rendering_rate,
                          camera,
                          0.75 * supereight_config_.mu);

// create image
  sensor_msgs::ImagePtr depth_render_msg(new sensor_msgs::Image());
  CreateImageMsg(image_pose_msg, depth_render_msg, computation_size_);
  depth_render_msg->encoding = "rgba8"; // rgba8 doesn't work
  depth_render_msg->is_bigendian = 0;
  memcpy((void *) depth_render_msg->data.data(), (void *) depth_render_, depth_render_msg->width
      * depth_render_msg->height * sizeof(float));

  sensor_msgs::ImagePtr track_render_msg(new sensor_msgs::Image());
  CreateImageMsg(image_pose_msg, track_render_msg, computation_size_);
  track_render_msg->encoding = "rgba8"; // rgba8 doesn't work
  track_render_msg->is_bigendian = 0;
  memcpy((void *) track_render_msg->data.data(), (void *) track_render_, track_render_msg->width
      * track_render_msg->height * sizeof(float));

  sensor_msgs::ImagePtr volume_render_msg(new sensor_msgs::Image());
  CreateImageMsg(image_pose_msg, volume_render_msg, computation_size_);
  volume_render_msg->encoding = "rgba8"; // rgba8 doesn't work
  volume_render_msg->is_bigendian = 0;
  memcpy((void *) volume_render_msg->data.data(), (void *) volume_render_, volume_render_msg->width
      * volume_render_msg->height * sizeof(float));

  // publish to topic
  depth_render_pub_.publish(*depth_render_msg);
  track_render_pub_.publish(*track_render_msg);
  volume_render_pub_.publish(*volume_render_msg);
#endif
  timings[7] = std::chrono::steady_clock::now();

  // ------------- supereight map visualization ------------

  if (std::is_same<FieldType, OFusion>::value) {
    visualizeMapOFusion(updated_blocks);
  } else if (std::is_same<FieldType, SDF>::value) {
    visualizeMapSDF(occupied_voxels, freed_voxels, updated_blocks);
  }

  timings[8] = std::chrono::steady_clock::now();

  Eigen::Vector3f tmp = pipeline_->getPosition();
  float3 pos = make_float3(tmp.x(), tmp.y(), tmp.z());
  storeStats(frame_, timings, pos, tracked, integrated);

  frame_++;
}

template<typename T>
void SupereightNode<T>::visualizeMapOFusion(std::vector<Eigen::Vector3i> updated_blocks) {
  // publish every N-th frame
  int N_frame_pub = 1;

  // get supereight map
  pipeline_->getMap(octree_);
  node_iterator<T> node_it(*octree_);

  //
  if (pub_map_update_) {
    visualization_msgs::Marker map_marker_msg;

    std::vector<Eigen::Vector3i> occupied_voxels = node_it.getOccupiedVoxels();

    map_marker_msg.header.frame_id = frame_id_;
    map_marker_msg.ns = frame_id_;
    map_marker_msg.id = 0;
    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
    map_marker_msg.scale.x = res_;
    map_marker_msg.scale.y = res_;
    map_marker_msg.scale.z = res_;
    map_marker_msg.action = visualization_msgs::Marker::ADD;
    map_marker_msg.color.r = 1.0f;
    map_marker_msg.color.g = 1.0f;
    map_marker_msg.color.b = 0.0f;
    map_marker_msg.color.a = 1.0;

    for (const auto &occupied_voxel : occupied_voxels) {
      geometry_msgs::Point cube_center;

      cube_center.x = ((double) occupied_voxel[0] + 0.5) * res_;
      cube_center.y = ((double) occupied_voxel[1] + 0.5) * res_;
      cube_center.z = ((double) occupied_voxel[2] + 0.5) * res_;

      map_marker_msg.points.push_back(cube_center);
    }

    if (frame_ % N_frame_pub == 0) {
      map_marker_pub_.publish(map_marker_msg);
    }
  }

  if (pub_block_based_) {
    visualization_msgs::Marker voxel_block_marker;
    voxel_block_marker.header.frame_id = frame_id_;
    voxel_block_marker.ns = frame_id_;
    voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
    voxel_block_marker.scale.x = res_;
    voxel_block_marker.scale.y = res_;
    voxel_block_marker.scale.z = res_;
    voxel_block_marker.action = visualization_msgs::Marker::ADD;
    voxel_block_marker.color.r = 0.0f;
    voxel_block_marker.color.g = 0.0f;
    voxel_block_marker.color.b = 1.0f;
    voxel_block_marker.color.a = 1.0;

    visualization_msgs::MarkerArray voxel_block_marker_array_msg;
    visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;

    if (pub_block_based_marker_) {
      voxel_block_marker_msg.id = 0;
      voxel_block_marker_msg.color.r = 1.0f;
      voxel_block_marker_msg.color.g = 0.0f;
      voxel_block_marker_msg.color.b = 0.0f;
      voxel_block_marker_msg.color.a = 1.0;
    }

    if (frame_ % N_frame_pub == 0) {
      for (const auto &updated_block : updated_blocks) {
        int morten_code =
            (int) compute_morton(updated_block[0], updated_block[1], updated_block[2]);

        std::vector<Eigen::Vector3i>
            occupied_block_voxels = node_it.getOccupiedVoxels(0.5, updated_block);

        if (pub_block_based_marker_array_) {
          voxel_block_marker.id = morten_code;
          voxel_block_marker.points.clear();

          for (const auto &occupied_voxel : occupied_block_voxels) {
            geometry_msgs::Point cube_center;
            cube_center.x = (static_cast<double>(occupied_voxel[0]) + 0.5) * res_;
            cube_center.y = (static_cast<double>(occupied_voxel[1]) + 0.5) * res_;
            cube_center.z = (static_cast<double>(occupied_voxel[2]) + 0.5) * res_;
            voxel_block_marker.points.push_back(cube_center);
          }
          voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
        }

        if (pub_block_based_marker_) {
          voxel_block_map_[morten_code] = occupied_block_voxels;
        }
      }

      if (pub_block_based_marker_array_) {
        block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
      }
    }

    if (pub_block_based_marker_) {
      for (auto voxel_block = voxel_block_map_.begin(); voxel_block != voxel_block_map_.end();
           voxel_block++) {
        for (const auto &occupied_voxel : voxel_block->second) {
          geometry_msgs::Point cube_center;

          cube_center.x = ((double) occupied_voxel[0] + 0.5) * res_;
          cube_center.y = ((double) occupied_voxel[1] + 0.5) * res_;
          cube_center.z = ((double) occupied_voxel[2] + 0.5) * res_;

          voxel_block_marker_msg.points.push_back(cube_center);
        }
      }
      block_based_marker_pub_.publish(voxel_block_marker_msg);
    }
  }
};

template<typename T>
void SupereightNode<T>::visualizeMapSDF(std::vector<Eigen::Vector3i> occupied_voxels,
                                        std::vector<Eigen::Vector3i> freed_voxels,
                                        std::vector<Eigen::Vector3i> updated_blocks) {
  // publish every N-th frame
  int N_frame_pub = 1;

  pipeline_->getMap(octree_);
  node_iterator<T> node_it(*octree_);

  if (pub_map_update_) {
    visualization_msgs::Marker map_marker_msg;

    std::vector<Eigen::Vector3i> surface_voxels = node_it.getSurfaceVoxels();

    map_marker_msg.header.frame_id = frame_id_;
    map_marker_msg.ns = frame_id_;
    map_marker_msg.id = 0;
    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
    map_marker_msg.scale.x = res_;
    map_marker_msg.scale.y = res_;
    map_marker_msg.scale.z = res_;
    map_marker_msg.action = visualization_msgs::Marker::ADD;
    map_marker_msg.color.r = 1.0f;
    map_marker_msg.color.g = 1.0f;
    map_marker_msg.color.b = 0.0f;
    map_marker_msg.color.a = 1.0;

    for (const auto &surface_voxel : surface_voxels) {
      geometry_msgs::Point cube_center;

      cube_center.x = ((double) surface_voxel[0] + 0.5) * res_;
      cube_center.y = ((double) surface_voxel[1] + 0.5) * res_;
      cube_center.z = ((double) surface_voxel[2] + 0.5) * res_;

      map_marker_msg.points.push_back(cube_center);
    }

    if (frame_ % N_frame_pub == 0) {
      map_marker_pub_.publish(map_marker_msg);
    }
  }

//  if (pub_block_based_) {
//    visualization_msgs::Marker voxel_block_marker;
//    voxel_block_marker.header.frame_id = frame_id_;
//    voxel_block_marker.ns = frame_id_;
//    voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
//    voxel_block_marker.scale.x = res_;
//    voxel_block_marker.scale.y = res_;
//    voxel_block_marker.scale.z = res_;
//    voxel_block_marker.action = visualization_msgs::Marker::ADD;
//    voxel_block_marker.color.r = 0.0f;
//    voxel_block_marker.color.g = 0.0f;
//    voxel_block_marker.color.b = 1.0f;
//    voxel_block_marker.color.a = 1.0;
//
//    visualization_msgs::MarkerArray voxel_block_marker_array_msg;
//    visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;
//
//    if (pub_block_based_marker_) {
//      voxel_block_marker_msg.id = 0;
//      voxel_block_marker_msg.color.r = 1.0f;
//      voxel_block_marker_msg.color.g = 0.0f;
//      voxel_block_marker_msg.color.b = 0.0f;
//      voxel_block_marker_msg.color.a = 1.0;
//    }
//
//    if (frame_ % N_frame_pub == 0) {
//      for (const auto &updated_block : updated_blocks) {
//        int morten_code = (int)compute_morton(
//            updated_block[0], updated_block[1], updated_block[2]);
//
//        std::vector<Eigen::Vector3i> occupied_block_voxels =
//            node_it.getSurfaceVoxels(0.25, updated_block);
//
//        if (pub_block_based_marker_array_) {
//          voxel_block_marker.id = morten_code;
//          voxel_block_marker.points.clear();
//
//          for (const auto &occupied_voxel : occupied_block_voxels) {
//            geometry_msgs::Point cube_center;
//            cube_center.x =
//                (static_cast<double>(occupied_voxel[0]) + 0.5) * res_;
//            cube_center.y =
//                (static_cast<double>(occupied_voxel[1]) + 0.5) * res_;
//            cube_center.z =
//                (static_cast<double>(occupied_voxel[2]) + 0.5) * res_;
//            voxel_block_marker.points.push_back(cube_center);
//          }
//          voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
//        }
//
//        if (pub_block_based_marker_) {
//          voxel_block_map_[morten_code] = occupied_block_voxels;
//        }
//      }
//
//      if (pub_block_based_marker_array_) {
//        block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
//      }
//    }
//
//    if (pub_block_based_marker_) {
//      for (auto voxel_block = voxel_block_map_.begin();
//           voxel_block != voxel_block_map_.end(); voxel_block++) {
//        for (const auto &occupied_voxel : voxel_block->second) {
//          geometry_msgs::Point cube_center;
//
//          cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
//          cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
//          cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;
//
//          voxel_block_marker_msg.points.push_back(cube_center);
//        }
//      }
//      block_based_marker_pub_.publish(voxel_block_marker_msg);
//    }
//  }
};

template<typename T>
double SupereightNode<T>::calculateAlpha(int64_t pre_time_stamp,
                                         int64_t post_time_stamp,
                                         int64_t img_time_stamp) {
  double alpha = (double) (img_time_stamp - pre_time_stamp) / (post_time_stamp - pre_time_stamp);
  return alpha;
}

template<typename T>
Eigen::Vector3f SupereightNode<T>::interpolateVector(const Eigen::Vector3f &pre_vector3D,
                                                     const Eigen::Vector3f &post_vector3D,
                                                     double alpha) {
  return pre_vector3D + alpha * (post_vector3D - pre_vector3D);
}

template<typename T>
Eigen::Quaternionf SupereightNode<T>::interpolateOrientation(const Eigen::Quaternionf &pre_orientation,
                                                             const Eigen::Quaternionf &post_orientation,
                                                             double alpha) {
  Eigen::Quaternionf int_orientation = pre_orientation.slerp(alpha, post_orientation);
  return int_orientation;
}

template<typename T>
Eigen::Matrix4f SupereightNode<T>::interpolatePose(const geometry_msgs::TransformStamped &pre_transformation,
                                                   const geometry_msgs::TransformStamped &post_transformation,
                                                   int64_t img_time_stamp) {
  double alpha = calculateAlpha(ros::Time(pre_transformation.header.stamp).toNSec(),
                                ros::Time(post_transformation.header.stamp).toNSec(),
                                img_time_stamp);

  Eigen::Vector3f pre_translation(pre_transformation.transform.translation.x,
                                  pre_transformation.transform.translation.y,
                                  pre_transformation.transform.translation.z);
  Eigen::Vector3f post_translation(post_transformation.transform.translation.x,
                                   post_transformation.transform.translation.y,
                                   post_transformation.transform.translation.z);

  Eigen::Quaternionf pre_rotation(pre_transformation.transform.rotation.w,
                                  pre_transformation.transform.rotation.x,
                                  pre_transformation.transform.rotation.y,
                                  pre_transformation.transform.rotation.z);
  Eigen::Quaternionf post_rotation(post_transformation.transform.rotation.w,
                                   post_transformation.transform.rotation.x,
                                   post_transformation.transform.rotation.y,
                                   post_transformation.transform.rotation.z);

  Eigen::Vector3f inter_translation = interpolateVector(pre_translation, post_translation, alpha);
  Eigen::Quaternionf inter_rotation = interpolateOrientation(pre_rotation, post_rotation, alpha);

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 1>(0, 3) = inter_translation;
  pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

  return pose;
}

/* Taken from https://github.com/ethz-asl/volumetric_mapping */
template<typename T>
std_msgs::ColorRGBA SupereightNode<T>::percentToColor(double h) {
  /* Helen's note: direct copy from OctomapProvider. */
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  /* blend over HSV-values (more colors) */

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f; /* if i is even */
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}
}  // namespace se

#endif  // SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP
