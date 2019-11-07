//
// Created by anna on 16/05/19.
//

#ifndef __SUPEREIGHT_ROS_CONFIG_HPP
#define __SUPEREIGHT_ROS_CONFIG_HPP

#include <ros/ros.h>

#include <se/config.h>



namespace se {
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

