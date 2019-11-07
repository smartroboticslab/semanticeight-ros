//
// Created by anna on 16/05/19.
//

#ifndef __SUPEREIGHT_ROS_CONFIG_HPP
#define __SUPEREIGHT_ROS_CONFIG_HPP

#include <Eigen/Dense>

#include <ros/ros.h>

#include <se/config.h>



namespace se {
  struct SupereightNodeConfig {
    bool enable_tracking;
    bool enable_rendering;
    bool block_based_map;
    Eigen::Vector2i input_size;
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

