/*
 * SPDX-FileCopyrightText: 2019 Anna Dai
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SUPEREIGHT_ROS_HPP
#define __SUPEREIGHT_ROS_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include <se/DenseSLAMSystem.h>

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
     */
    void fusionCallback();

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
    //std_msgs::ColorRGBA percentToColor(double h);



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
    std::unique_ptr<uint32_t> input_rgb_;
    std::unique_ptr<uint32_t> rgb_render_;
    std::unique_ptr<uint32_t> depth_render_;
    std::unique_ptr<uint32_t> track_render_;
    std::unique_ptr<uint32_t> volume_render_;

    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;

    // Publishers
    ros::Publisher supereight_pose_pub_;

    // Render publishers
    ros::Publisher rgb_render_pub_;
    ros::Publisher depth_render_pub_;
    ros::Publisher volume_render_pub_;
    ros::Publisher track_render_pub_;

    // Visualization publishers
    ros::Publisher map_marker_pub_;
    ros::Publisher block_based_marker_pub_;
    ros::Publisher boundary_marker_pub_;
    ros::Publisher frontier_marker_pub_;

    // Circular buffers for incoming messages
    boost::circular_buffer<geometry_msgs::TransformStamped> pose_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      depth_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      rgb_buffer_;
    std::mutex pose_buffer_mutex_;
    std::mutex depth_buffer_mutex_;
    std::mutex rgb_buffer_mutex_;

    std::mutex fusion_mutex_;

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

    // Timings
    std::vector<std::chrono::time_point<std::chrono::steady_clock> > timings_;
    std::vector<std::string> timing_labels_;
  };
} // namespace se

#endif // SUPEREIGHT_ROS_HPP

