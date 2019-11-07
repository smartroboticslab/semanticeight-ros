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

#ifndef __SUPEREIGHT_ROS_HPP
#define __SUPEREIGHT_ROS_HPP

#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <ratio>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <getopt.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>

// supereight package
#include <se/DenseSLAMSystem.h>
#include <se/config.h>
#include <se/node_iterator.hpp>
#include <se/utils/morton_utils.hpp>

// supereight_ros headers
#include "supereight_ros/ImagePose.h"  //message
#include "supereight_ros/CircularBuffer.hpp"
#include "supereight_ros/supereight_ros_config.hpp"



//typedef std::map<int, vec3i, std::less<int>,
//    Eigen::aligned_allocator<std::pair<const int, vec3i> > >
//    mapvec3i;

namespace se {
  class SupereightNode {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SupereightNode(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

    ~SupereightNode() {
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
    void readConfig(const ros::NodeHandle& nh_private);

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
     * @brief set TF matrix supereight map from world
     * @param[in] translation first pose from robot
     * @param[in] init_position_octree: volume size vector * initial pos factor
     * vector
     * @param[out] const_translation: first pose intializet in the octree map
     * @param[out] tf_matrix
     * @return bool
     */
    bool setTFMapfromWorld(const Eigen::Vector3f &translation,
                           const Eigen::Vector3f &init_position_octree,
                           Eigen::Vector3f       &const_translation,
                           Eigen::Matrix4f       &tf_matrix);

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
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);

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
    //void visualizeMapOFusion(vec3i &updated_blocks,
    //                         vec3i &frontier_blocks,
    //                         map3i &frontier_blocks_map,
    //                         vec3i &occlusion_blocks);

    // TODO: change SDF visualization to be block based
    /**
     * @brief loads the SDF map and publishs it to a ros topic
     * @param updated_blocks
     */
    //void visualizeMapSDF(vec3i &occupied_voxels,
    //                     vec3i &freed_voxels,
    //                     vec3i &updated_blocks);

    /* Taken from https://github.com/ethz-asl/volumetric_mapping */
    std_msgs::ColorRGBA percentToColor(double h);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    SupereightNodeConfig node_config_;

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
    uint32_t* depth_render_ = nullptr;
    uint32_t* volume_render_ = nullptr;
    uint32_t* track_render_ = nullptr;
#endif

    Eigen::Vector2i computation_size_;
    float res_;
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

    // voxel blockwise update for visualization
    //mapvec3i voxel_block_map_;
    //mapvec3i surface_voxel_map_;
    //mapvec3i frontier_voxel_map_;
    //mapvec3i occlusion_voxel_map_;
    // block based visualization
    bool pub_map_update_ = false;
    bool pub_block_based_ = true;

    bool enable_icp_tracking_;
    bool use_test_image_;
    bool set_world_to_map_tf_ = false;

    Eigen::Matrix4f tf_map_from_world_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tf_world_from_map_ = Eigen::Matrix4f::Identity();

    Eigen::Vector3f init_position_octree_;
    Eigen::Vector3f const_translation_;
    std::ofstream myfile;
  };
} // namespace se

#endif // SUPEREIGHT_ROS_HPP

