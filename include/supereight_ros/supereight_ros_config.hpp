// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __SUPEREIGHT_ROS_CONFIG_HPP
#define __SUPEREIGHT_ROS_CONFIG_HPP

#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>

#include "se/config.h"
#include "supereight_ros/control_interface.hpp"
#include "supereight_ros/dataset.hpp"



namespace se {
/*!
 * \brief Configuration for se::SupereightNode
 */
struct SupereightNodeConfig {
    /*!
     * Use the ICP tracking from supereight to compute the camera pose.
     */
    bool enable_tracking;

    /*!
     * Generate and publish the depth, RGB, tracking info and volume render
     * images from supereight.
     */
    bool enable_rendering;

    /*!
     * Add RGB images to the supereight pipeline.
     * This has no effect in the supereight map for the time being since
     * supereight uses only depth images. The option is available to make it
     * easier to use modified supereight versions that make use of RGB images.
     */
    bool enable_rgb;

    /** Enable reconstruction of individual objects using semantic information.
     */
    bool enable_objects;

    /*!
     * The resolution of the input depth and RGB images (width, height).
     *
     * \warning This should be set to the correct image resolution since it is
     * used to initialize supereight.
     */
    Eigen::Vector2i input_res;

    /*!
     * The type of the pose topic. Valid values are `geometry_msgs::PoseStamped`
     * and `geometry_msgs::TransformStamped`.
     */
    std::string pose_topic_type;

    /*!
     * The number of elements in the pose circular buffer.
     *
     * \note This should in general be at least an order of magnitude larger
     * than se::SupereightNodeConfig::depth_buffer_size and
     * se::SupereightNodeConfig::rgb_buffer_size.
     */
    int pose_buffer_size;

    /*!
     * The number of elements in the depth image circular buffer. A low value
     * will result in integrating fairly recent frames into the map. A high
     * value will result in integrating older frames into the map. A value of 1
     * will result in integrating the most recent depth frame into the map.
     */
    int depth_buffer_size;

    /*!
     * The number of elements in the RGB image circular buffer.
     */
    int rgb_buffer_size;

    /*!
     * The maximum time difference in seconds between a depth and RGB image to
     * consider them matched.
     */
    double max_timestamp_diff;

    /*!
     * Whether to center the world frame to the position of the first received
     * pose. This only has an effect if
     * se::SupereightNodeConfig::enable_tracking is false.
     */
    bool center_at_first_position;

    /*!
     * The rate at which the 3D map visualization in rviz is updated. A value of
     * 0 disables the 3D visualization completely.
     */
    int visualization_rate;

    /*!
     * The maximum z coordinates of voxels that are being published as visualization messages.
     * Useful to remove the roof of the generated map. The default value is INFINITY, no limit.
     */
    float visualization_max_z;

    /*!
     * The type of the dataset being run.
     */
    Dataset dataset;

    /*!
     * Whether to visualize the results of 360-degree entropy and depth raycasting. This is helpful
     * for debugging but may significantly decrease performance.
     */
    bool visualize_360_raycasting;

    /** Set to NaN for no time limit.
     */
    double max_exploration_time;

    /** Run Mask R-CNN on the input images.
     */
    bool run_segmentation;

    /** The control interface to use.
     */
    se::ControlInterface control_interface;
};



/*!
 * \brief Read an se::SupereightNodeConfig from a ROS node's parameters.
 *
 * \param[in] nh The handle of the ROS node containing the parameters.
 *
 * \return An se::SupereightNodeConfig.
 */
SupereightNodeConfig read_supereight_node_config(const ros::NodeHandle& nh);



/*!
 * \brief Print the values of an se::SupereightNodeConfig using ROS_INFO.
 *
 * \param[in] config The se::SupereightNodeConfig struct to print.
 */
void print_supereight_node_config(const SupereightNodeConfig& config);



/*!
 * \brief Read an se::Configuration from a ROS node's parameters.
 *
 * \param[in] nh The handle of the ROS node containing the parameters.
 *
 * \return An se::Configuration.
 */
Configuration read_supereight_config(const ros::NodeHandle& nh);

} // namespace se

#endif
