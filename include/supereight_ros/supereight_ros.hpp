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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "se/DenseSLAMSystem.h"
#include "se/octree_iterator.hpp"

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

    void SemClassCallback(const sensor_msgs::ImageConstPtr& class_msg);

    void SemInstanceCallback(const sensor_msgs::ImageConstPtr& instance_msg);

    geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::PoseStamped&  T_WB_msg) const;

    void visualizeWholeMap();

    void visualizeObjects();

    void visualizeFrontiers();

    void visualizeCandidates();

    void visualizeCandidatePaths();

    void visualizeRejectedCandidates();

    void visualizeGoal();

    void visualizeMAV();

    bool is_free(const se::Volume<VoxelImpl::VoxelType>& volume) const;

    bool is_occupied(const se::Volume<VoxelImpl::VoxelType>& volume) const;

    geometry_msgs::TransformStamped T_MW_Msg() const;

    geometry_msgs::TransformStamped T_BC_Msg() const;

    /* \brief Return a translation from the Habitat-Sim world frame to the supereight world frame.
     * This functions is used to map the initial pose in Habitat-Sim into the identity pose in
     * supereight-ros.
     * \warning This function blocks until a single pose is received from Habitat-Sim.
     */
    geometry_msgs::TransformStamped T_WWh_Msg() const;

    visualization_msgs::Marker mapDimMsg() const;



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
    Eigen::Vector3f init_t_WB_;
    Eigen::Vector2i image_res_;
    int frame_;
    int num_planning_iterations_;

    // Image buffers
    std::unique_ptr<float>    input_depth_;
    std::unique_ptr<uint32_t> input_rgba_;
    std::unique_ptr<uint32_t> rgba_render_;
    std::unique_ptr<uint32_t> depth_render_;
    std::unique_ptr<uint32_t> track_render_;
    std::unique_ptr<uint32_t> volume_render_;

    // Semantics
    se::SegmentationResult input_segmentation_;

    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber class_sub_;
    ros::Subscriber instance_sub_;

    // Publishers
    ros::Publisher supereight_pose_pub_;
    tf2_ros::TransformBroadcaster pose_tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    ros::Publisher path_pub_;

    // Render publishers
    ros::Publisher depth_render_pub_;
    ros::Publisher rgba_render_pub_;
    ros::Publisher volume_render_pub_;
    ros::Publisher track_render_pub_;

    // Visualization publishers
    ros::Publisher map_dim_pub_;
    ros::Publisher map_free_pub_;
    ros::Publisher map_occupied_pub_;
    ros::Publisher map_unknown_pub_;
    ros::Publisher map_object_pub_;
    ros::Publisher map_frontier_pub_;
    ros::Publisher map_candidate_pub_;
    ros::Publisher map_candidate_path_pub_;
    ros::Publisher map_rejected_candidate_pub_;
    ros::Publisher map_goal_pub_;
    ros::Publisher mav_sphere_pub_;

    // Visualization colors
    const Eigen::Vector4f color_occupied_ = Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
    const Eigen::Vector4f color_free_ = Eigen::Vector4f(0.0, 1.0, 0.0, 0.5);
    const Eigen::Vector4f color_unknown_ = Eigen::Vector4f(0.0, 0.0, 0.0, 0.5);
    const Eigen::Vector4f color_object_ = Eigen::Vector4f(1.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4f color_frontier_ = Eigen::Vector4f(1.0, 0.5, 0.0, 0.5);
    const Eigen::Vector4f color_candidate_ = Eigen::Vector4f(1.0, 1.0, 0.0, 1.0);
    const Eigen::Vector4f color_candidate_path_ = Eigen::Vector4f(1.0, 1.0, 0.0, 1.0);
    const Eigen::Vector4f color_rejected_candidate_ = Eigen::Vector4f(1.0, 0.0, 0.0, 0.5);
    const Eigen::Vector4f color_goal_ = Eigen::Vector4f(1.0, 0.0, 1.0, 1.0);
    const Eigen::Vector4f color_mav_sphere_ = Eigen::Vector4f(0.0, 0.0, 1.0, 0.5);

    // Circular buffers for incoming messages
    boost::circular_buffer<geometry_msgs::TransformStamped> pose_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      depth_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      rgb_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      class_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr>      instance_buffer_;
    std::mutex pose_buffer_mutex_;
    std::mutex depth_buffer_mutex_;
    std::mutex rgb_buffer_mutex_;
    std::mutex class_buffer_mutex_;
    std::mutex instance_buffer_mutex_;

    std::mutex fusion_mutex_;

    /*!
     * Global/map coordinate frame. Will always look up TF transforms to this
     * frame.
     */
    const std::string world_frame_id_;
    const std::string map_frame_id_;
    const std::string body_frame_id_;
    const std::string camera_frame_id_;

    // Constant messages
    visualization_msgs::Marker map_dim_msg_;

    // Timings
    std::vector<std::chrono::time_point<std::chrono::steady_clock> > timings_;
    std::vector<std::string> timing_labels_;
  };

} // namespace se

#endif // SUPEREIGHT_ROS_HPP

