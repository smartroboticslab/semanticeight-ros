// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __SUPEREIGHT_ROS_HPP
#define __SUPEREIGHT_ROS_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "se/DenseSLAMSystem.h"

#include "supereight_ros/supereight_ros_config.hpp"



namespace se {

  /*!
   * \brief A ROS node that wraps a supereight pipeline.
   */
  class SupereightNode {
  public:
    SupereightNode(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

    /*!
     * \brief Integrate the measurements using the supereight pipeline.
     *
     * First try to match an RGB image (if `se::SupereightNode::enable_rgb
     * == true`) and a pose (if `se::SupereightNode::enable_tracking == false`)
     * to the oldest depth image. If matching fails, the return
     * without performing a map update.
     *
     * If matching is successful call the supereight pipeline `preprocess`,
     * `track`, `integrate` and `raycast` stages to update the map. If rendering
     * is enabled (if `se::SupereightNode::enable_rendering == true`) also
     * generate the depth, RGBA, tracking and volume renders and publish them in
     * the `/supereight/rgba_render`, `/supereight/depth_render`,
     * `/supereight/track_render` and `/supereight/volume_render` topics
     * respectively.
     */
    void runPipelineOnce();

    /*!
     * \brief Save the current supereight map to a `.vtk` file.
     *
     * \note The map is only saved if the value of
     * se::Configuration::dump_volume_file has been set to a non-empty string.
     */
    void saveMap();

    /*!
     * \brief Access the supereight pipeline directly if needed.
     *
     * \return An std::shared_ptr to the supereight pipeline.
     */
    std::shared_ptr<DenseSLAMSystem> getSupereightPipeline() {
      return pipeline_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



  private:
    /*!
     * \brief Read the supereight and se::SupereightNode YAML configuration
     * files.
     *
     * See se::SupereightNodeConfig for details about the se::SupereightNode
     * configuration.
     */
    void readConfig(const ros::NodeHandle& nh_private);

    /*!
     * \brief Set up the ROS publishers and subscribers.
     *
     * This function should only be called from the constructor.
     **/
    void setupRos();

    geometry_msgs::TransformStamped T_MW_Msg();

    geometry_msgs::TransformStamped T_BC_Msg();

    visualization_msgs::Marker mapDimMsg();

    /*!
     * \brief ROS callback for depth image messages in topic
     * `/camera/depth_image`.
     *
     * Append the depth image to se::SupereightNode::depth_buffer_.     *
     *
     * \param[in] depth_msg The received depth image message.
     */
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);

    /*!
     * \brief ROS callback for RGB image messages in topic `/camera/rgb_image`.
     *
     * Appends the RGB image to se::SupereightNode::rgb_buffer_.
     *
     * \param[in] rgb_msg The received RGB image message.
     */
    void RGBCallback(const sensor_msgs::ImageConstPtr& rgb_msg);

    /*!
     * \brief ROS callback for body pose messages of type
     * geometry_msgs::PoseStamped in topic `/pose`.
     *
     * Converts the pose to an Eigen::Matrix4d and calls
     * se::SupereightNode::poseCallback.
     *
     * \param[in] T_WB_msg The received body pose message in the world frame
     *                     with the ROS convention (x forward, y left, z up).
     */
    void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& T_WB_msg);

    /*!
     * \brief ROS callback for body pose messages of type
     * geometry_msgs::TransformStamped in topic `/pose`.
     *
     * Converts the pose to an Eigen::Matrix4d and calls
     * se::SupereightNode::poseCallback.
     *
     * \param[in] T_WB_msg The received body pose message in the world frame
     *                     with the ROS convention (x forward, y left, z up).
     */
    void transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& T_WB_msg);

    /*!
     * \brief Generic callback for body pose messages.
     *
     * Called by se::SupereightNode::transformStampedCallback or
     * se::SupereightNode::poseStampedCallback, depending on the message type.
     * Convert the body pose (using the ROS convention x forward, y left, z up)
     * to a camera pose (using the supereight convention x right, y down, z
     * forward) and append it to se::SupereightNode::pose_buffer_.     *
     *
     * \param[in] T_WB   The received body pose in the world frame with the ROS
     *                   convention (x forward, y left, z up).
     * \param[in] header The header of the body pose message.
     */
    void poseCallback(const Eigen::Matrix4d&  T_WB,
                      const std_msgs::Header& header);

    /*
     * \brief loads the occpuancy map and publishes it to a ros topic
     * \param updated_blocks
     */
    //void visualizeMapOFusion(vec3i &updated_blocks,
    //                         vec3i &frontier_blocks,
    //                         map3i &frontier_blocks_map,
    //                         vec3i &occlusion_blocks);

    /*
     * \brief loads the SDF map and publishes it to a ros topic
     * \param updated_blocks
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
    SensorImpl sensor_;
    std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;
    Eigen::Vector3f t_MW_;
    Eigen::Matrix4f T_CB_;
    Eigen::Vector2i image_res_;
    int frame_;

    // Image buffers
    std::unique_ptr<float>    input_depth_;
    std::unique_ptr<uint32_t> input_rgba_;
    std::unique_ptr<uint32_t> rgba_render_;
    std::unique_ptr<uint32_t> depth_render_;
    std::unique_ptr<uint32_t> track_render_;
    std::unique_ptr<uint32_t> volume_render_;

    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;

    // Publishers
    ros::Publisher supereight_pose_pub_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Render publishers
    ros::Publisher depth_render_pub_;
    ros::Publisher rgba_render_pub_;
    ros::Publisher volume_render_pub_;
    ros::Publisher track_render_pub_;

    // Visualization publishers
    ros::Publisher map_dim_pub_;
    //ros::Publisher map_marker_pub_;
    //ros::Publisher block_based_marker_pub_;
    //ros::Publisher boundary_marker_pub_;
    //ros::Publisher frontier_marker_pub_;

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

    /*!
     * Global/map coordinate frame. Will always look up TF transforms to this
     * frame.
     */
    const std::string world_frame_id_;
    const std::string map_frame_id_;
    const std::string body_frame_id_;
    const std::string camera_frame_id_;

    // Timings
    std::vector<std::chrono::time_point<std::chrono::steady_clock> > timings_;
    std::vector<std::string> timing_labels_;
  };

} // namespace se

#endif // SUPEREIGHT_ROS_HPP

