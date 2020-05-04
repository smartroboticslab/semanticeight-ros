// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include <cstdlib>
#include <memory>
#include <signal.h>

#include "supereight_ros/supereight_ros.hpp"



// Needed due to the way the TICK and TOCK macros are defined.
PerfStats Stats;

volatile static bool keep_running = true;

static void signal_handler(int) {
  keep_running = false;
}




int main(int argc, char **argv) {
  signal(SIGINT, signal_handler);

  // Initialize the ROS node
  ros::init(argc, argv, "supereight_ros_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::unique_ptr<se::SupereightNode> node (new se::SupereightNode(nh, nh_private));

  while (keep_running) {
    ros::spinOnce();
  }

  // Save the map to a file if needed
  node->saveMap();

  exit(EXIT_SUCCESS);
}

