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

#ifndef __SUPEREIGHT_ROS_HPP
#define __SUPEREIGHT_ROS_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include <se/DenseSLAMSystem.h>

#include "supereight_ros/ImagePose.h"
#include "supereight_ros/CircularBuffer.hpp"
#include "supereight_ros/supereight_ros_config.hpp"



namespace se {
  class SupereightNode {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SupereightNode(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

    ~SupereightNode();

    /**
     * @brief access for external packages
     * @return pointer to supereight pipeline
     */
    std::shared_ptr<DenseSLAMSystem> getSupereightPipeline() {
      return pipeline_;
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



  private:
    /**
     * @brief sets configuration from YAML file to nodehandle
     * definitions, see supereight/se/config.h
     */
    void readConfig(const ros::NodeHandle& nh_private);

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

    /** Add an RGB image to the queue.
     * \param[in] rgb_msg Pointer to the RGB image message.
     */
    void RGBCallback(const sensor_msgs::ImageConstPtr& rgb_msg);

    /**
     * @brief reads camera info
     * @param camInfoIn
     */
    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr &camInfoIn);

    /**
     * @brief adds pose from ground truth file or recorded pose to pose buffer
     * @param T_WR_msg
     *
     * The supplied pose is the camera frame expressed in world coordinates.
     * The camera frame is using the ROS convention of x forward, z up.
     */
    void poseCallback(const geometry_msgs::TransformStamped::ConstPtr& T_WR_msg);

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



    // ROS node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    SupereightNodeConfig node_config_;

    // Supereight
    Configuration supereight_config_;
    std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;
    Eigen::Vector3f init_position_octree_;
    Eigen::Vector2i computation_size_;
    int frame_;
    float res_;
    //std::shared_ptr<se::Octree<SE_VOXEL_IMPLEMENTATION> > octree_;

    // Image buffers
    std::unique_ptr<uint16_t> input_depth_;
    std::unique_ptr<uint32_t> depth_render_;
    std::unique_ptr<uint32_t> track_render_;
    std::unique_ptr<uint32_t> volume_render_;

    // Camera info
    sensor_msgs::CameraInfo CamInfo;
    image_geometry::PinholeCameraModel CamModel;
    bool cam_info_ready_ = false;

    // Subscribers
    ros::Subscriber image_sub_;
    ros::Subscriber rgb_image_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber image_pose_sub_;
    ros::Subscriber cam_info_sub_;

    // Publishers
    ros::Publisher image_pose_pub_;
    ros::Publisher supereight_pose_pub_;

    // Render publishers
    ros::Publisher depth_render_pub_;
    ros::Publisher volume_render_pub_;
    ros::Publisher track_render_pub_;

    // Visualization publishers
    ros::Publisher map_marker_pub_;
    ros::Publisher block_based_marker_pub_;
    ros::Publisher boundary_marker_pub_;
    ros::Publisher frontier_marker_pub_;

    /**
    * buffer and quque for incoming data streams, in case the matching can't
    * be immediately done.
    */
    CircularBuffer<geometry_msgs::TransformStamped> pose_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> depth_image_buffer_;
    static constexpr size_t depth_image_buffer_size_ = 50;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> rgb_image_buffer_;
    static constexpr size_t rgb_image_buffer_size_ = 50;

    // voxel blockwise update for visualization
    //mapvec3i voxel_block_map_;
    //mapvec3i surface_voxel_map_;
    //mapvec3i frontier_voxel_map_;
    //mapvec3i occlusion_voxel_map_;
    // block based visualization
    //bool pub_map_update_ = false;

    /**
     * Global/map coordinate frame. Will always look up TF transforms to this
     * frame.
     */
    std::string frame_id_;

    bool set_world_to_map_tf_ = false;
    Eigen::Matrix4f tf_map_from_world_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tf_world_from_map_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f const_translation_;

    // Timings
    std::vector<std::chrono::time_point<std::chrono::steady_clock> > timings_;
    std::vector<std::string> timing_labels_;
  };
} // namespace se

#endif // SUPEREIGHT_ROS_HPP

