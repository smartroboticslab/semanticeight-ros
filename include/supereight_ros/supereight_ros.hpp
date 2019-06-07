//
// Created by anna on 04/04/19.
//

/**
 * Coordinate Frames Definition
 * robot poses are in 'world frame'
 * 'octree frame' is positioned on the lower corner of the volume, so that the whole volume is
 * contained in the positive octant
 * transform from 'map frame' to 'world frame' is defined arbritrarily.
 * axes of the map and the octree frame are aligned at the origin of the 'map frame' in the
 * octree coordinates is at the point of the volume encoded in init_pose
 */

/**
* @brief
* @param
* @returns
**/

#ifndef SUPEREIGHT_ROS_HPP
#define SUPEREIGHT_ROS_HPP

#include <stdint.h>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <queue>
#include <ratio>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <getopt.h>
#include <glog/logging.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include <Eigen/StdVector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eigen_conversions/eigen_msg.h>

#include <image_geometry/pinhole_camera_model.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/MarkerArray.h>

#include <set>

// supereight package

#include <se/DenseSLAMSystem.h>
#include <se/config.h>
#include <se/node_iterator.hpp>
#include <se/utils/morton_utils.hpp>

// supereight_ros headers
#include "supereight_ros/ImagePose.h"  //message
#include "supereight_ros/CircularBuffer.hpp"
#include "supereight_ros/supereight_utils.hpp"
#include "supereight_ros/functions.hpp"

//typedef  std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i>>  vec3i;
typedef std::map<int,
                 vec3i,
                 std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, vec3i> > > map3i;

namespace se {

template<typename T>
class SupereightNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SupereightNode> Ptr;

  SupereightNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  virtual ~SupereightNode() {
    free(input_depth_);
#ifdef WITH_RENDERING
    free(depth_render_);
    free(track_render_);
    free(volume_render_);
#endif
  }

  /**
  * @brief sets configuration from YAML file to nodehandle
   * definitions, see supereight/se/config.h
  **/
  void setSupereightConfig(const ros::NodeHandle &nh_private);

  /**
   * @brief access for external packages
   * @return pointer to supereight pipeline
   */
  std::shared_ptr<DenseSLAMSystem> getSupereightPipeline() {
    return pipeline_;
  }

  /**
   * set rviz visualization type
   * @param pub_block_based fastest way, currently only for OFusion
   */

  void setSupereightVisualizationMapBased(const bool &pub_block_based) {
    pub_block_based_ = pub_block_based;
  }

  /**
   * @brif prints configuration parameters of the supereight denseSLAM pipeline
   * @param config
   */
  void printSupereightConfig(const Configuration &config);
/**
 * @brief set TF matrix supereight map from world
 * @param[in] translation first pose from robot
 * @param[in] init_position_octree: volume size vector * initial pos factor vector
 * @param[out] const_translation: first pose intializet in the octree map
 * @param[out] tf_matrix
 * @return bool
 */
  bool setTFMapfromWorld(const Eigen::Vector3f &translation,
                         const Eigen::Vector3f &init_position_octree,
                         Eigen::Vector3f &const_translation,
                         Eigen::Matrix4f &tf_matrix);
  // public variables
  Eigen::Vector3f init_pose_;

  bool use_tf_transforms_;

 private:
  /**
 * @brief Sets up publishing and subscribing, should only be called from
 * constructor
 **/
  void setupRos();

  /**
   * @brief adds images to image queue
   * @param old_image_msg
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &image_msg);

/**
 * @brief reads camera info
 * @param camInfoIn
 */

  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr &camInfoIn);
  /**
   * @brief adds pose from ground truth file or recorded pose to pose buffer
   * @param pose_msg
   */
  void poseCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_msg);

  /**
   * @brief aligns the pose with the image and calls the supereight denseslam
   * pipeline
   * @param image_pose_msg
   */
  void fusionCallback(const supereight_ros::ImagePose::ConstPtr &image_pose_msg);

  /**
   * @brief loads the occpuancy map and publishs it to a ros topic
   * @param updated_blocks
   */
  void visualizeMapOFusion(std::vector<Eigen::Vector3i> &updated_blocks, std::vector<Eigen::Vector3i> &frontier_blocks);
//  void visualizeMapOFusion(vec3i &updated_blocks, vec3i &frontier_blocks);

  // TODO: change SDF visualization to be block based
  /**
   * @brief loads the SDF map and publishs it to a ros topic
   * @param updated_blocks
   */
//  void visualizeMapSDF(vec3i &occupied_voxels, vec3i &freed_voxels, vec3i &updated_blocks);
  void visualizeMapSDF(std::vector<Eigen::Vector3i> &occupied_voxels, std::vector<Eigen::Vector3i> &freed_voxels, std::vector<Eigen::Vector3i> &updated_blocks);
  /**
   * @brief       Calculates the fraction a given sample is located between the
   * closest pre and post sample
   *
   * @param[in]   pre_time_stamp    Closest pre vicon time stamp to image
   * @param[in]   post_time_stamp   Closest post vicon time stamp to image
   * @param[in]   img_time_stamp    Image time stamp
   *
   * @return      Fraction
   */
  double calculateAlpha(int64_t pre_time_stamp, int64_t post_time_stamp, int64_t img_time_stamp);

  /**
   * @brief      Linear 3D interpolation
   *
   * @param[in]  pre_vector3D  The pre vector 3D
   * @param[in]  post_vector3D      The post vector 3D
   * @param[in]  alpha              Fraction
   *
   * @return     Interpolated translation
   */
  Eigen::Vector3f interpolateVector(const Eigen::Vector3f &pre_vector3D,
                                    const Eigen::Vector3f &post_vector3D,
                                    double alpha);

  /**
   * @brief      Slerp interpolation for quaterniond
   *
   * @param[in]  pre_orientation       The previous orientation
   * @param[in]  post_orientation      The post orientation
   * @param[in]  alpha                 Fraction
   *
   * @return     Interpolated orientation
   */
  Eigen::Quaternionf interpolateOrientation(const Eigen::Quaternionf &pre_orientation,
                                            const Eigen::Quaternionf &post_orientation,
                                            double alpha);

  /**
   * @brief      Interpolation for transformations
   *
   * @param[in]  pre_transformation       The previous transformation
   * @param[in]  post_transformation      The post transformation
   * @param[in]  alpha                    Fraction
   *
   * @return     Interpolated transformation
   */
  Eigen::Matrix4f interpolatePose(const geometry_msgs::TransformStamped &pre_transformation,
                                  const geometry_msgs::TransformStamped &post_transformation,
                                  int64_t img_time_stamp);

  /* Taken from https://github.com/ethz-asl/volumetric_mapping */
  std_msgs::ColorRGBA percentToColor(double h);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string frame_id_;

  // get pipeline with map
  std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;

//#ifdef MAP_OM
  std::shared_ptr<se::Octree<OFusion>> octree_ = nullptr;
//#elif MAP_SDF
//  std::shared_ptr<se::Octree<SDF>> octree_ = nullptr;
//#endif
  // pipeline configuration
  Configuration supereight_config_;
  Eigen::Vector2i image_size_;
  int frame_;

  uint16_t *input_depth_ = nullptr;

#ifdef WITH_RENDERING
  uchar4 *depth_render_ = nullptr;
  uchar4 *volume_render_ = nullptr;
  uchar4 *track_render_ = nullptr;
#endif

  Eigen::Vector2i computation_size_;
  double res_;
  int occupied_voxels_sum_;
  float
      cam_baseline_ = 0.110f; // [m] from rotors_description/urdf/component_snippets.xacro vi_sensor

  // simulation camera reader
  sensor_msgs::CameraInfo CamInfo;
  image_geometry::PinholeCameraModel CamModel;

  bool cam_info_ready_ = false;

  // Subscriber
  ros::Subscriber image_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber image_pose_sub_;
  ros::Subscriber cam_info_sub_;

  // Publisher
  ros::Publisher image_pose_pub_;
  ros::Publisher supereight_pose_pub_;
  ros::Publisher gt_tf_pose_pub_;

#ifdef WITH_RENDERING
  ros::Publisher depth_render_pub_;
  ros::Publisher volume_render_pub_;
  ros::Publisher track_render_pub_;
#endif

  // Visualization
  ros::Publisher map_marker_pub_;
  ros::Publisher block_based_marker_pub_;
  ros::Publisher boundary_marker_pub_;
  ros::Publisher frontier_marker_pub_;
  /**
  * buffer and quque for incoming data streams, in case the matching can't
  * be immediately done.
  */
  CircularBuffer<geometry_msgs::TransformStamped> pose_buffer_;
  std::deque<sensor_msgs::Image> image_queue_;

  // timing
  std::deque<std::chrono::time_point<std::chrono::system_clock>> stop_watch_;
  uint64_t image_time_stamp_;

  // voxel blockwise update for visualization
  map3i voxel_block_map_;
  map3i surface_voxel_map_;
  map3i frontier_voxel_map_;
  map3i occlusion_voxel_map_;
  // block based visualization
  bool pub_map_update_ = false;
  bool pub_block_based_ = true;

  bool enable_icp_tracking_;

  // transform pose
//  tf::TransformListener tf_listener_;
//  tf::StampedTransform gt_pose_transform_;

  bool set_world_to_map_tf_ = false;
  // get latet transform to the planning frame and transform the pose
//  tf::StampedTransform stf_octree_to_depth_cam_;//
//  geometry_msgs::TransformStamped::Ptr tfs_octree_to_map;
//
//  tf::StampedTransform stf_depth_cam_to_octree_;
//  geometry_msgs::TransformStamped::Ptr tfs_map_to_octree_;


  Eigen::Matrix4f tf_map_from_world_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_world_from_map_ = Eigen::Matrix4f::Identity();

  Eigen::Vector3f init_position_octree_;
  Eigen::Vector3f const_translation_;
  std::ofstream myfile;

};

}  // namespace se


#include "supereight_ros/supereight_ros_impl.hpp"
#endif  // SUPEREIGHT_ROS_HPP
