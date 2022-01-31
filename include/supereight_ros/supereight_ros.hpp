// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __SUPEREIGHT_ROS_HPP
#define __SUPEREIGHT_ROS_HPP

#include <Eigen/Dense>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <mav_interface_msgs/AutopilotStatusService.h>
#include <mav_interface_msgs/conversions.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

#include "se/DenseSLAMSystem.h"
#include "se/exploration_planner.hpp"
#include "se/octree_iterator.hpp"
#ifdef SE_WITH_MASKRCNN
#    include "maskrcnn_trt/maskrcnn.hpp"
#endif // SE_WITH_MASKRCNN

#include "supereight_ros/supereight_ros_config.hpp"



namespace se {

/*!
 * \brief A ROS node that wraps a supereight pipeline.
 */
class SupereightNode {
    public:
    SupereightNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~SupereightNode();

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
    void matchAndFuse();

    void fuse(const Eigen::Matrix4f& T_WC,
              const sensor_msgs::ImageConstPtr& depth_image,
              const sensor_msgs::ImageConstPtr& color_image,
              const se::SegmentationResult& segmentation,
              const ros::Time& depth_timestamp,
              const bool out_of_order_fusion = false);

    void plan();

    /*!
     * \brief Save the current supereight map to a `.vtk` file.
     *
     * \note The map is only saved if the value of
     * se::Configuration::dump_volume_file has been set to a non-empty string.
     */
    void saveMap();

    int saveCandidates();

    /*!
     * \brief Access the supereight pipeline directly if needed.
     *
     * \return An std::shared_ptr to the supereight pipeline.
     */
    std::shared_ptr<DenseSLAMSystem> getSupereightPipeline()
    {
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
    void poseCallback(const Eigen::Matrix4d& T_WB, const std_msgs::Header& header);

    void SemClassCallback(const sensor_msgs::ImageConstPtr& class_msg);

    void SemInstanceCallback(const sensor_msgs::ImageConstPtr& instance_msg);

    void visualizeWholeMap();

    void visualizeMapMesh();

    void visualizeObjects();

    void visualizeObjectMeshes();

    void visualizeObjectAABBs();

    void visualizeFrontiers();

    void visualizeCandidates();

    void visualizeGoal();

    void visualizeMAV();

    void visualizePoseHistory();

    void visualizePoseGridHistory();

    void visualizeEnvironmentAABB();

    void visualizeSamplingAABB();

    template<typename VoxelImplT>
    bool is_free(const se::Volume<typename VoxelImplT::VoxelType>& volume) const
    {
        constexpr bool is_tsdf = VoxelImplT::invert_normals;
        return (is_tsdf && volume.data.x > 0.0f) || (!is_tsdf && volume.data.x < 0.0f);
    }

    template<typename VoxelImplT>
    bool is_occupied(const se::Volume<typename VoxelImplT::VoxelType>& volume) const
    {
        constexpr bool is_tsdf = VoxelImplT::invert_normals;
        return (is_tsdf && volume.data.x <= 0.0f) || (!is_tsdf && volume.data.x > 0.0f);
    }

    geometry_msgs::TransformStamped T_WM_Msg() const;

    geometry_msgs::TransformStamped T_BC_Msg() const;

    visualization_msgs::Marker mapDimMsg() const;

    void runNetwork(const Eigen::Matrix4f& T_WC,
                    const sensor_msgs::ImageConstPtr& depth_image,
                    const sensor_msgs::ImageConstPtr& color_image,
                    const ros::Time& depth_timestamp);



    // ROS node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    SupereightNodeConfig node_config_;

    // Supereight
    Configuration supereight_config_;
    SensorImpl sensor_;
    std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;
    std::unique_ptr<se::ExplorationPlanner> planner_;
    Eigen::Matrix4f T_MW_;
    Eigen::Matrix4f T_WM_;
    Eigen::Matrix4f T_CB_;
    Eigen::Vector3f init_t_WB_;
    Eigen::Vector2i image_res_;
    std::atomic_int frame_;
    int num_planning_iterations_;
    int num_failed_planning_iterations_;
    int max_failed_planning_iterations_;
    std::chrono::time_point<std::chrono::steady_clock> exploration_start_time_;

    // Image buffers
    std::unique_ptr<float> input_depth_;
    std::unique_ptr<uint32_t> input_rgba_;
    std::unique_ptr<uint32_t> rgba_render_;
    std::unique_ptr<uint32_t> depth_render_;
    std::unique_ptr<uint32_t> track_render_;
    std::unique_ptr<uint32_t> volume_render_;
    std::unique_ptr<uint32_t> volume_render_color_;
    std::unique_ptr<uint32_t> volume_render_scale_;
    std::unique_ptr<uint32_t> volume_render_min_scale_;
    std::unique_ptr<uint32_t> class_render_;
    std::unique_ptr<uint32_t> instance_render_;
    std::unique_ptr<uint32_t> raycast_render_;

    // Semantics
    se::SegmentationResult input_segmentation_;
#ifdef SE_WITH_MASKRCNN
    mr::MaskRCNNConfig network_config_;
    mr::MaskRCNN network_;
#endif // SE_WITH_MASKRCNN

    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber class_sub_;
    ros::Subscriber instance_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Publishers
    ros::Publisher supereight_pose_pub_;
    tf2_ros::TransformBroadcaster pose_tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    ros::Publisher path_pub_;

    // Render publishers
    ros::Publisher depth_render_pub_;
    ros::Publisher rgba_render_pub_;
    ros::Publisher track_render_pub_;
    ros::Publisher volume_render_pub_;
    ros::Publisher volume_render_color_pub_;
    ros::Publisher volume_render_scale_pub_;
    ros::Publisher volume_render_min_scale_pub_;
    ros::Publisher class_render_pub_;
    ros::Publisher instance_render_pub_;
    ros::Publisher raycast_render_pub_;
    ros::Publisher entropy_render_pub_;
    ros::Publisher entropy_depth_render_pub_;

    // Visualization publishers
    ros::Publisher map_pub_;
    ros::Publisher object_pub_;
    ros::Publisher map_frontier_pub_;
    ros::Publisher map_candidate_pub_;
    ros::Publisher map_goal_pub_;
    ros::Publisher mav_vis_pub_;
    ros::Publisher pose_history_pub_;
    ros::Publisher limit_pub_;

    // ROS service Client
    ros::ServiceClient mav_status_service_;

    // Visualization colors
    const Eigen::Vector4f color_occupied_ = Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
    const Eigen::Vector4f color_free_ = Eigen::Vector4f(0.0, 1.0, 0.0, 0.5);
    const Eigen::Vector4f color_unknown_ = Eigen::Vector4f(0.0, 0.0, 0.0, 0.5);
    const Eigen::Vector4f color_object_ = Eigen::Vector4f(1.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4f color_frontier_ = Eigen::Vector4f(1.0, 0.5, 0.0, 0.5);
    const Eigen::Vector4f color_candidate_ = Eigen::Vector4f(1.0, 1.0, 0.0, 1.0);
    const Eigen::Vector4f color_rejected_candidate_ = Eigen::Vector4f(1.0, 0.0, 0.0, 0.25);
    const Eigen::Vector4f color_goal_ = Eigen::Vector4f(1.0, 0.0, 1.0, 1.0);
    const Eigen::Vector4f color_mav_sphere_ = Eigen::Vector4f(0.0, 0.0, 1.0, 0.25);
    const Eigen::Vector4f color_pose_history_ = Eigen::Vector4f(0.5, 0.5, 0.5, 0.5);
    const Eigen::Vector4f color_sampling_aabb_ = Eigen::Vector4f(1.0, 0.0, 0.0, 0.5);

    // Circular buffers for incoming messages
    boost::circular_buffer<geometry_msgs::TransformStamped> pose_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> depth_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> rgb_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> class_buffer_;
    boost::circular_buffer<sensor_msgs::ImageConstPtr> instance_buffer_;
    std::mutex pose_buffer_mutex_;
    std::mutex depth_buffer_mutex_;
    std::mutex rgb_buffer_mutex_;
    std::mutex class_buffer_mutex_;
    std::mutex instance_buffer_mutex_;

    std::mutex matching_mutex_;
    std::mutex fusion_mutex_;
    std::mutex map_mutex_;
    std::mutex pose_mutex_;
    std::mutex network_mutex_;

    // Threads
    std::thread matching_thread_;
    std::thread planning_thread_;
    std::atomic_bool keep_running_;

    /*!
     * Global/map coordinate frame. Will always look up TF transforms to this
     * frame.
     */
    std::string world_frame_id_;
    std::string map_frame_id_;
    std::string body_frame_id_;
    std::string camera_frame_id_;

    // Constant messages
    visualization_msgs::Marker map_dim_msg_;

    // Statistics
    // Statistics for a single frame.
    typedef std::map<std::string, double> FrameStats;
    // Statistics for a all frames for a single section, e.g. Planning.
    typedef std::vector<FrameStats> SectionStats;
    // Statistics for all sections.
    typedef std::map<std::string, SectionStats> Stats;
    // Valid statistic names for each section in the order in which they should be printed.
    typedef std::map<std::string, std::vector<std::string>> StatNames;

    const StatNames stat_names_ = {
        {"Fusion", {"Frame",       "Timestamp",       "Preprocessing",
                    "Tracking",    "Integration",     "Object integration",
                    "Rendering",   "Visualization",   "Total",
                    "Free volume", "Occupied volume", "Explored volume",
                    "RAM usage",   "t_WB x",          "t_WB y",
                    "t_WB z",      "q_WB x",          "q_WB y",
                    "q_WB z",      "q_WB w"}},
        {"Planning",
         {"Planning iteration",
          "Timestamp",
          "Planning time",
          "Goal utility",
          "Goal entropy gain",
          "Goal LoD gain",
          "Goal path time",
          "Goal t_WB x",
          "Goal t_WB y",
          "Goal t_WB z",
          "Goal q_WB x",
          "Goal q_WB y",
          "Goal q_WB z",
          "Goal q_WB w"}},
        {"Network", {"Timestamp", "Network time"}}};
    Stats stats_;
    std::map<std::string, std::string> stat_tsv_filenames_;
    mutable std::mutex stat_mutex_;

    double start_time_;
    void initStats();
    void newStatFrame(const std::string& section);
    void sampleStat(const std::string& section, const std::string& stat, double value);
    void sampleTime(const std::string& section,
                    const std::string& stat,
                    double value = ros::WallTime::now().toSec());
    double getStat(const std::string& section, const std::string& stat) const;
    std::vector<double> getLastStats(const std::string& section) const;
    void printStats() const;
    void writeFrameStats(const std::string& section) const;
};

} // namespace se

#endif // SUPEREIGHT_ROS_HPP
