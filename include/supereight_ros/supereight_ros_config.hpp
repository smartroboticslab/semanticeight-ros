/*
 * SPDX-FileCopyrightText: 2019 Anna Dai
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SUPEREIGHT_ROS_CONFIG_HPP
#define __SUPEREIGHT_ROS_CONFIG_HPP

#include <Eigen/Dense>

#include <ros/ros.h>

#include <se/config.h>



namespace se {
  struct SupereightNodeConfig {
    bool enable_tracking;
    bool enable_rendering;
    bool enable_rgb;
    Eigen::Vector2i input_size;
    int pose_buffer_size;
    int depth_buffer_size;
    int rgb_buffer_size;
    double max_timestamp_diff;
  };



  SupereightNodeConfig read_supereight_node_config(const ros::NodeHandle& nh);



  void print_supereight_node_config(const SupereightNodeConfig& config);



  /**
   * @brief Read the supereight configuration into a struct given a node
   * handle.
   */
  Configuration read_supereight_config(const ros::NodeHandle& nh);



  /**
   * @brief Show the value of the supplied supereight configuration struct
   * using ROS_INFO.
   */
  void print_supereight_config(const Configuration& config);

} // namespace se

#endif

