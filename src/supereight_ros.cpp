//
// Created by anna on 12/04/19.
//

#include <iostream>
#include <memory>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <lodepng.h>
//#include <se/default_parameters.h>
#include <supereight_ros/supereight_ros.hpp>

namespace se {

std::string sep = "\n----------------------------------------\n";



SupereightNode::SupereightNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      pose_buffer_(500),
      image_size_{640, 480},
      frame_(0),
      frame_id_("map"),
      enable_icp_tracking_(false),
      use_tf_transforms_(true),
      occupied_voxels_sum_(0) {

  // Configure supereight_config with default values
  readConfig(nh_private);

  input_depth_ = (uint16_t *) malloc(sizeof(uint16_t) * image_size_.x() * image_size_.y());
  init_pose_ = supereight_config_.initial_pos_factor.cwiseProduct(supereight_config_.volume_size);
  computation_size_ = image_size_ / supereight_config_.compute_size_ratio;

#ifdef WITH_RENDERING
  const size_t render_size_bytes
      = sizeof(uint32_t) * computation_size_.x() * computation_size_.y();
  depth_render_  = static_cast<uint32_t*>(malloc(render_size_bytes));
  track_render_  = static_cast<uint32_t*>(malloc(render_size_bytes));
  volume_render_ = static_cast<uint32_t*>(malloc(render_size_bytes));
#endif

  pipeline_ = std::shared_ptr<DenseSLAMSystem>(new DenseSLAMSystem(
      computation_size_,
      Eigen::Vector3i::Constant(supereight_config_.volume_resolution.x()),
      Eigen::Vector3f::Constant(supereight_config_.volume_size.x()),
      init_pose_,
      supereight_config_.pyramid,
      supereight_config_));

  //pipeline_->getMap(octree_);
  res_ = static_cast<float>(pipeline_->getModelDimensions().x())
       / static_cast<float>(pipeline_->getModelResolution().x());

  setupRos();
}



void SupereightNode::setupRos() {
  // Subscriber
  image_sub_ = nh_.subscribe("/camera/depth_image", 100, &SupereightNode::depthCallback, this);
  pose_sub_ = nh_.subscribe("/pose", 1000, &SupereightNode::poseCallback, this);
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
  boundary_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("boundary_marker", 1);
  frontier_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("frontier_marker", 1);

  std::string ns = ros::this_node::getName();
  bool read_from_launch = ros::param::get(ns + "/enable_icp_tracking", enable_icp_tracking_);
  ros::param::get(ns + "/use_tf_transforms", use_tf_transforms_);
  bool use_test_image = ros::param::get(ns + "/use_test_image", use_test_image_);
}

// Read config.yaml into Configuration class
void SupereightNode::readConfig(const ros::NodeHandle& nh_private) {
  supereight_config_ = read_supereight_config(nh_private);

  // Read the node configuration.
  std::vector<int> image_size_vector;
  if (nh_private.getParam("input_size", image_size_vector)) {
    for (unsigned int i = 0; i < image_size_vector.size(); i++) {
      image_size_[i] = image_size_vector[i];
    }
  }

  init_position_octree_ = supereight_config_.initial_pos_factor.cwiseProduct(supereight_config_.volume_size);
  bool supereight_visualization_block = true;
  if (nh_private.getParam("block_based_map", supereight_visualization_block)) {
    setSupereightVisualizationMapBased(supereight_visualization_block);
    printf("map viz %i", supereight_visualization_block);
  } else {
    setSupereightVisualizationMapBased(supereight_visualization_block);
  }
};



void SupereightNode::depthCallback(
    const sensor_msgs::ImageConstPtr& depth_msg) {

  // Allocate new depth image message
  sensor_msgs::ImagePtr test_image(new sensor_msgs::Image());
  createImageMsg(depth_msg, test_image);
  test_image->data.resize(test_image->height * test_image->step, 0.0f);

  if (use_test_image_) {
    // Generate the test image and push back to the image queue.
    createTestImage(depth_msg, test_image);
    image_queue_.push_back(*test_image);
  } else {
    // Just push back to the image queue.
    image_queue_.push_back(*depth_msg);
  }
}



void SupereightNode::camInfoCallback(const sensor_msgs::CameraInfoConstPtr &camInfoIn) {
//  ROS_INFO("cam info callback %i", cam_info_ready_);
  if (cam_info_ready_) return;
  CamInfo = *camInfoIn;
  CamModel.fromCameraInfo(camInfoIn);

  cam_info_ready_ = true;
  ROS_INFO("got camera model..");
//  std::cout << CamModel.cameraInfo() << std::endl;
//  std::cout << CamModel.fx() << "," << CamModel.fy() << std::endl;
}

void SupereightNode::poseCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_msg) {
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
  pose = swapAxes(pose);

  // initialize map from world transformation matrix
  if (!set_world_to_map_tf_) {
//    const_translation_ = init_position_octree_;
//    const_translation_(0) -= translation(0);
//    const_translation_(1) -= translation(2);
//    const_translation_(2) -= translation(1);
//    // x- axis 90, z-axis 90
//    tf_map_from_world_ << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
//    tf_map_from_world_.block<3, 1>(0, 3) -= init_position_octree_;
    set_world_to_map_tf_ = setTFMapfromWorld(translation,
                                             init_position_octree_,
                                             const_translation_,
                                             tf_map_from_world_);
    std::cout << "world to map tf \n" << translation << " \n" << tf_map_from_world_ << sep;
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

  uint64_t oldest_depth_timestamp;
  if (!image_queue_.empty()) {
    oldest_depth_timestamp = ros::Time(image_queue_.front().header.stamp).toNSec();
  }
  // insert to ring buffer
  pose_buffer_.put(pose_tf_msg_);
  while (image_queue_.size() > 0
      && (ros::Time(pose_msg->header.stamp).toNSec() > oldest_depth_timestamp)) {
    supereight_ros::ImagePose image_pose_msg;

    image_pose_msg.image = image_queue_.front();

    geometry_msgs::TransformStamped pre_pose;
    geometry_msgs::TransformStamped post_pose;

    pose_buffer_.get(oldest_depth_timestamp, pre_pose, post_pose);

    image_pose_msg.pre_pose = pre_pose;
    image_pose_msg.post_pose = post_pose;

    // fusion callback
    image_pose_pub_.publish(image_pose_msg);

    image_queue_.pop_front();

    if (!image_queue_.empty())
      oldest_depth_timestamp = ros::Time(image_queue_.front().header.stamp).toNSec();
  }

  //publish the transformed vi_sensor_ground truth
  geometry_msgs::PoseStamped gt_tf_pose_msg;
  gt_tf_pose_msg.header = pose_msg->header;
  gt_tf_pose_msg.header.frame_id = frame_id_;
  gt_tf_pose_msg.pose.position.x = pose_tf_msg_.transform.translation.x + init_position_octree_(0);
  gt_tf_pose_msg.pose.position.y = pose_tf_msg_.transform.translation.y + init_position_octree_(0);
  gt_tf_pose_msg.pose.position.z = pose_tf_msg_.transform.translation.z + init_position_octree_(0);
  gt_tf_pose_msg.pose.orientation = pose_tf_msg_.transform.rotation;
  gt_tf_pose_pub_.publish(gt_tf_pose_msg);
}

void SupereightNode::fusionCallback(const supereight_ros::ImagePose::ConstPtr &image_pose_msg) {
  bool integrated = false;
  bool raycasted = false;
  bool tracked = false;
// ROS_INFO("fusion callback");
  // creates file with poses
//  myfile.open("/home/anna/Data/integration.txt", std::ofstream::app);

  std::chrono::time_point<std::chrono::steady_clock> timings[9];

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
  const Eigen::Matrix4f gt_pose = interpolate_pose(
      image_pose_msg->pre_pose,
      image_pose_msg->post_pose,
      ros::Time(image_pose_msg->image.header.stamp).toNSec());
//  myfile<<std::fixed<<gt_pose<<std::endl;
//  myfile.close();

  //-------- supereight access point ----------------
  timings[2] = std::chrono::steady_clock::now();
  // supereight takes the values in [mm]
  pipeline_->preprocessing(input_depth_, image_size_, supereight_config_.bilateral_filter);
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
  tf::pointEigenToMsg(tracked_pose.block<3, 1>(0, 3).cast<double>(), supereight_pose.pose.position);
  Eigen::Quaternionf q_rot(tracked_pose.block<3, 3>(0, 0));
  tf::quaternionEigenToMsg(q_rot.cast<double>(), supereight_pose.pose.orientation);

// publish poses estimated using supereight map
  supereight_pose_pub_.publish(supereight_pose);

  timings[4] = std::chrono::steady_clock::now();

  // for visualization
  //vec3i occupied_voxels(0);
  //vec3i freed_voxels(0);
  //vec3i updated_blocks(0);
  //vec3i frontier_blocks(0);
  //vec3i occlusion_blocks(0);
  //map3i frontier_blocks_map;
  //map3i occlusion_blocks_map;
//  std::cout<< "sizes of frontier blocks  " << frontier_blocks.size() << " freed " << freed_voxels
//      .size() << " updated " << updated_blocks.size() << std::endl;

  // Integrate only if tracking was successful or it is one of the
  // first 4 frames.
  if (tracked || frame_ <= 3) {
    integrated = pipeline_->integration(camera,
                                        supereight_config_.integration_rate,
                                        supereight_config_.mu,
                                        frame_);
                                        //&updated_blocks,
                                        //&frontier_blocks,
                                        //&occlusion_blocks,
                                        //frontier_blocks_map,
                                        //occlusion_blocks_map);

  } else {
    integrated = false;
  }
  //std::cout << "sizes of frontier blocks: " << frontier_blocks.size() << std::endl;
//  std::cout << "frontier map " << frontier_blocks_map.size() << " updated " << updated_blocks.size() << std::endl;

  timings[5] = std::chrono::steady_clock::now();

//  pipeline_->raycasting(camera, supereight_config_.mu, frame_);
  timings[6] = std::chrono::steady_clock::now();

  ROS_INFO("integrated %i, tracked %i ", integrated, tracked);
  //ROS_INFO_STREAM("occluded voxels = " << occlusion_blocks.size());
//    ROS_INFO_STREAM( "freed_voxels = " << freed_voxels.size());
  //ROS_INFO_STREAM("updated voxels  = " << updated_blocks.size());

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
  createImageMsg(image_pose_msg, depth_render_msg, computation_size_);
  depth_render_msg->encoding = "rgba8"; // rgba8 doesn't work
  depth_render_msg->is_bigendian = 0;
  memcpy((void *) depth_render_msg->data.data(), (void *) depth_render_, depth_render_msg->width
      * depth_render_msg->height * sizeof(float));

  sensor_msgs::ImagePtr track_render_msg(new sensor_msgs::Image());
  createImageMsg(image_pose_msg, track_render_msg, computation_size_);
  track_render_msg->encoding = "rgba8"; // rgba8 doesn't work
  track_render_msg->is_bigendian = 0;
  memcpy((void *) track_render_msg->data.data(), (void *) track_render_, track_render_msg->width
      * track_render_msg->height * sizeof(float));

  sensor_msgs::ImagePtr volume_render_msg(new sensor_msgs::Image());
  createImageMsg(image_pose_msg, volume_render_msg, computation_size_);
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

  //if (std::is_same<FieldType, OFusion>::value) {
  //  visualizeMapOFusion(updated_blocks, frontier_blocks, frontier_blocks_map, occlusion_blocks);
  //} else if (std::is_same<FieldType, SDF>::value) {
  //  visualizeMapSDF(occupied_voxels, freed_voxels, updated_blocks);
  //}

  timings[8] = std::chrono::steady_clock::now();
  Eigen::Vector3f tmp = pipeline_->getPosition();
  float3 pos = make_float3(tmp.x(), tmp.y(), tmp.z());
  //storeStats(frame_, timings, pos, tracked, integrated);
  //frontier_blocks.clear();
  //updated_blocks.clear();
  frame_++;

}

//void SupereightNode::visualizeMapOFusion(vec3i &updated_blocks,
//                                            vec3i &frontier_blocks,
//                                            map3i &frontier_blocks_map,
//                                            vec3i &occlusion_blocks) {
////void SupereightNode::visualizeMapOFusion(std::vector<Eigen::Vector3i>& updated_blocks,
////                                            std::vector<Eigen::Vector3i>& frontier_blocks) {
//  // publish every N-th frame
//  int N_frame_pub = 1;
//
//  // get supereight map
//  pipeline_->getMap(octree_);
//  node_iterator<T> node_it(*octree_);
//
//  // set with stored morton code
////  std::set<uint64_t> surface_voxel_set;
////  std::set<uint64_t> frontier_voxel_set;
////  std::set<uint64_t> occlusion_voxel_set;
//
////  bool getExplorationArea =
////      pipeline_->getExplorationCandidate(surface_voxel_set, occlusion_voxel_set);
////                                                               frontier_voxel_set,
////                                                               occlusion_voxel_set);
////  if (!getExplorationArea) { ROS_ERROR("no exploration area received "); }
//
//  visualization_msgs::Marker voxel_block_marker;
//  voxel_block_marker.header.frame_id = frame_id_;
//  voxel_block_marker.ns = frame_id_;
//  voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
//  voxel_block_marker.scale.x = res_;
//  voxel_block_marker.scale.y = res_;
//  voxel_block_marker.scale.z = res_;
//  voxel_block_marker.action = visualization_msgs::Marker::ADD;
//  voxel_block_marker.color.r = 0.0f;
//  voxel_block_marker.color.g = 0.0f;
//  voxel_block_marker.color.b = 1.0f;
//  voxel_block_marker.color.a = 1.0;
//
//  visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;
//  voxel_block_marker_msg.id = 0;
//  voxel_block_marker_msg.ns = "occlusion";
//  voxel_block_marker_msg.color.r = 1.0f;
//  voxel_block_marker_msg.color.g = 0.0f;
//  voxel_block_marker_msg.color.b = 0.0f;
//  voxel_block_marker_msg.color.a = 1.0;
//
//  visualization_msgs::Marker surface_voxels_msg = voxel_block_marker;
//  surface_voxels_msg.id = 0;
//  surface_voxels_msg.ns = "surface frontier";
//  surface_voxels_msg.lifetime = ros::Duration(6);
//  surface_voxels_msg.color.r = 1.0f;
//  surface_voxels_msg.color.g = 0.0f;
//  surface_voxels_msg.color.b = 1.0f;
//  surface_voxels_msg.color.a = 0.5;
//  visualization_msgs::Marker frontier_voxels_msg = voxel_block_marker;
//  frontier_voxels_msg.ns = "frontier";
//  frontier_voxels_msg.id = 0;
////  frontier_voxels_msg.lifetime = ros::Duration(10);
//  frontier_voxels_msg.color.r = 0.0f;
//  frontier_voxels_msg.color.g = 1.0f;
//  frontier_voxels_msg.color.b = 0.0f;
//  frontier_voxels_msg.color.a = 0.5f;
//
//  visualization_msgs::Marker occlusion_voxels_msg = voxel_block_marker;
//  occlusion_voxels_msg.ns = "occluded surface";
//  occlusion_voxels_msg.id = 0;
////  occlusion_voxels_msg.lifetime = ros::Duration(10);
//  occlusion_voxels_msg.color.r = 1.0f;
//  occlusion_voxels_msg.color.g = 1.0f;
//  occlusion_voxels_msg.color.b = 0.0f;
//  occlusion_voxels_msg.color.a = 0.5;
//
//  if (frame_ % N_frame_pub == 0) {
//    for (const auto &updated_block : updated_blocks) {
//      int morton_code = (int) compute_morton(updated_block[0], updated_block[1], updated_block[2]);
////      ROS_INFO("morton code updated %i ", morton_code );
////      std::cout << "updated block " << updated_block << std::endl;
//      vec3i occupied_block_voxels = node_it.getOccupiedVoxels(0.65, updated_block);
////      std::cout << "size occp blocks " << occupied_block_voxels.size() << std::endl;
//      voxel_block_map_[morton_code] = occupied_block_voxels;
//    }
//
//
//    /**
//     * FRONTIER
//     */
//    for (const auto &frontier_block : frontier_blocks) {
//      int morton_code =
//          (int) compute_morton(frontier_block[0], frontier_block[1], frontier_block[2]);
////      ROS_INFO("morton code updated %i ", morton_code );
////      std::cout << "frontier block \n" << frontier_block << std::endl;
//
//      vec3i frontier_block_voxels = node_it.getFrontierVoxels(0.1f, frontier_block);
////      std::cout << "size frontier blocks " << frontier_block_voxels.size() << std::endl;
//      frontier_voxel_map_[morton_code] = frontier_block_voxels;
//    }
//    for (const auto &frontier_block : frontier_blocks_map) {
//      vec3i frontier_block_voxels = node_it.getFrontierVoxels(0.1f, frontier_block.second);
//      surface_voxel_map_[frontier_block.first] = frontier_block_voxels;
//    }
//    for (const auto &occl_block : occlusion_blocks) {
//      int morton_code = (int) compute_morton(occl_block[0], occl_block[1], occl_block[2]);
//      vec3i occl_block_voxels = node_it.getOccludedVoxels(0.1f, occl_block);
//      occlusion_voxel_map_[morton_code] = occl_block_voxels;
//    }
//    /**
//     * SURFACE
//     */
///*    for (auto it = surface_voxel_set.begin(); it != surface_voxel_set.end(); ++it) {
////      std::cout << *it << std::endl;
//      Eigen::Vector3i surface_blocks = unpack_morton(*it);
////      std::cout << "surface block \n"<< surface_blocks << std::endl;
//      // TODO problem with getting occupied voxels
//      surface_voxel_map_[*it] = node_it.getOccupiedVoxels(0.5, surface_blocks);
////      if(frontier_voxel_set.find(*it)!=frontier_voxel_set.end()){
////        ROS_INFO("%i also in frontier ", *it);
////      }// morton code
//    }*/
//
////    for(auto it = frontier_voxel_set.begin(); it!= frontier_voxel_set.end(); ++it){
////      Eigen::Vector3i frontier_blocks = unpack_morton(*it);
////      frontier_voxel_map_[*it] = node_it.getOccupiedVoxels(0.1, frontier_blocks);
////    }
//
//  }
//
//  for (auto voxel_block = voxel_block_map_.begin(); voxel_block != voxel_block_map_.end();
//       voxel_block++) {
//    for (const auto &occupied_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) occupied_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) occupied_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) occupied_voxel[2] + 0.5) * res_;
//
//      voxel_block_marker_msg.points.push_back(cube_center);
//    }
//  }
//  for (auto voxel_block = surface_voxel_map_.begin(); voxel_block != surface_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &surface_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) surface_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) surface_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) surface_voxel[2] + 0.5) * res_;
//
//      surface_voxels_msg.points.push_back(cube_center);
//    }
//  }
//
//  for (auto voxel_block = frontier_voxel_map_.begin(); voxel_block != frontier_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &frontier_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) frontier_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) frontier_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) frontier_voxel[2] + 0.5) * res_;
//
//      frontier_voxels_msg.points.push_back(cube_center);
//    }
//  }
//
//  for (auto voxel_block = occlusion_voxel_map_.begin(); voxel_block != occlusion_voxel_map_.end();
//       voxel_block++) {
//    for (const auto &occl_voxel : voxel_block->second) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) occl_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) occl_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) occl_voxel[2] + 0.5) * res_;
//
//      occlusion_voxels_msg.points.push_back(cube_center);
//    }
//  }
//  boundary_marker_pub_.publish(surface_voxels_msg);
//  frontier_marker_pub_.publish(frontier_voxels_msg);
//  frontier_marker_pub_.publish(occlusion_voxels_msg);
//  block_based_marker_pub_.publish(voxel_block_marker_msg);
//};

//void SupereightNode::visualizeMapSDF(vec3i &occupied_voxels,
//                                        vec3i &freed_voxels,
//                                        vec3i &updated_blocks) {
////void SupereightNode::visualizeMapSDF(std::vector<Eigen::Vector3i>& occupied_voxels,
////                                        std::vector<Eigen::Vector3i>& freed_voxels,
////                                        std::vector<Eigen::Vector3i>& updated_blocks) {
//  // publish every N-th frame
//  int N_frame_pub = 1;
//
//  pipeline_->getMap(octree_);
//  node_iterator<T> node_it(*octree_);
//
//  if (pub_map_update_) {
//    visualization_msgs::Marker map_marker_msg;
//
//    vec3i surface_voxels = node_it.getSurfaceVoxels();
//
//    map_marker_msg.header.frame_id = frame_id_;
//    map_marker_msg.ns = frame_id_;
//    map_marker_msg.id = 0;
//    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
//    map_marker_msg.scale.x = res_;
//    map_marker_msg.scale.y = res_;
//    map_marker_msg.scale.z = res_;
//    map_marker_msg.action = visualization_msgs::Marker::ADD;
//    map_marker_msg.color.r = 1.0f;
//    map_marker_msg.color.g = 1.0f;
//    map_marker_msg.color.b = 0.0f;
//    map_marker_msg.color.a = 1.0;
//
//    for (const auto &surface_voxel : surface_voxels) {
//      geometry_msgs::Point cube_center;
//
//      cube_center.x = ((double) surface_voxel[0] + 0.5) * res_;
//      cube_center.y = ((double) surface_voxel[1] + 0.5) * res_;
//      cube_center.z = ((double) surface_voxel[2] + 0.5) * res_;
//
//      map_marker_msg.points.push_back(cube_center);
//    }
//
//    if (frame_ % N_frame_pub == 0) {
//      map_marker_pub_.publish(map_marker_msg);
//    }
//  }
//
////  if (pub_block_based_) {
////    visualization_msgs::Marker voxel_block_marker;
////    voxel_block_marker.header.frame_id = frame_id_;
////    voxel_block_marker.ns = frame_id_;
////    voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
////    voxel_block_marker.scale.x = res_;
////    voxel_block_marker.scale.y = res_;
////    voxel_block_marker.scale.z = res_;
////    voxel_block_marker.action = visualization_msgs::Marker::ADD;
////    voxel_block_marker.color.r = 0.0f;
////    voxel_block_marker.color.g = 0.0f;
////    voxel_block_marker.color.b = 1.0f;
////    voxel_block_marker.color.a = 1.0;
////
////    visualization_msgs::MarkerArray voxel_block_marker_array_msg;
////    visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;
////
////    if (pub_block_based_marker_) {
////      voxel_block_marker_msg.id = 0;
////      voxel_block_marker_msg.color.r = 1.0f;
////      voxel_block_marker_msg.color.g = 0.0f;
////      voxel_block_marker_msg.color.b = 0.0f;
////      voxel_block_marker_msg.color.a = 1.0;
////    }
////
////    if (frame_ % N_frame_pub == 0) {
////      for (const auto &updated_block : updated_blocks) {
////        int morten_code = (int)compute_morton(
////            updated_block[0], updated_block[1], updated_block[2]);
////
////        std::vector<Eigen::Vector3i> occupied_block_voxels =
////            node_it.getSurfaceVoxels(0.25, updated_block);
////
////        if (pub_block_based_marker_array_) {
////          voxel_block_marker.id = morten_code;
////          voxel_block_marker.points.clear();
////
////          for (const auto &occupied_voxel : occupied_block_voxels) {
////            geometry_msgs::Point cube_center;
////            cube_center.x =
////                (static_cast<double>(occupied_voxel[0]) + 0.5) * res_;
////            cube_center.y =
////                (static_cast<double>(occupied_voxel[1]) + 0.5) * res_;
////            cube_center.z =
////                (static_cast<double>(occupied_voxel[2]) + 0.5) * res_;
////            voxel_block_marker.points.push_back(cube_center);
////          }
////          voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
////        }
////
////        if (pub_block_based_marker_) {
////          voxel_block_map_[morten_code] = occupied_block_voxels;
////        }
////      }
////
////      if (pub_block_based_marker_array_) {
////        block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
////      }
////    }
////
////    if (pub_block_based_marker_) {
////      for (auto voxel_block = voxel_block_map_.begin();
////           voxel_block != voxel_block_map_.end(); voxel_block++) {
////        for (const auto &occupied_voxel : voxel_block->second) {
////          geometry_msgs::Point cube_center;
////
////          cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
////          cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
////          cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;
////
////          voxel_block_marker_msg.points.push_back(cube_center);
////        }
////      }
////      block_based_marker_pub_.publish(voxel_block_marker_msg);
////    }
////  }
//};



/* Taken from https://github.com/ethz-asl/volumetric_mapping */
std_msgs::ColorRGBA SupereightNode::percentToColor(double h) {
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

bool SupereightNode::setTFMapfromWorld(const Eigen::Vector3f &translation,
                                          const Eigen::Vector3f &init_position_octree,
                                          Eigen::Vector3f &const_translation,
                                          Eigen::Matrix4f &tf_matrix) {
  const_translation = init_position_octree;
  const_translation(0) -= translation(0);
  const_translation(1) -= translation(2);
  const_translation(2) -= translation(1);
  // x- axis 90, z-axis 90
  tf_matrix << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  tf_matrix.block<3, 1>(0, 3) += init_position_octree;
  return true;
}
}  // namespace se

