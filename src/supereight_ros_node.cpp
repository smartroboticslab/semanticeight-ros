/*
 * SPDX-FileCopyrightText: 2019 Anna Dai
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdlib>

#include "supereight_ros/supereight_ros.hpp"



// Needed due to the way the TICK and TOCK macros are defined.
PerfStats Stats;



int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "supereight_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  se::SupereightNode node(nh, nh_private);
  ROS_INFO("Initialization finished");

  ros::spin();

  exit(EXIT_SUCCESS);
}

