#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

import cv2 as cv
import numpy as np

import cv_bridge
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image



# Instantiate a single CvBridge object for all conversions
_bridge = cv_bridge.CvBridge()

def generate_test_image(
        width: int,
        height: int,
        near_wall_m: float,
        far_wall_m: float) -> Image:
    # Generate the depth image
    depth = np.full((height, width, 1), near_wall_m, dtype=np.float32)
    depth[:,0:width//2,:] = far_wall_m
    # Convert to a ROS message
    msg = _bridge.cv2_to_imgmsg(depth, "32FC1")
    msg.header.stamp = rospy.get_rostime()
    return msg

def generate_test_pose() -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "world"
    return msg

if __name__ == "__main__":
    try:
        # Settings
        width = 640
        height = 480
        near_wall_m = 2.0
        far_wall_m = 8.0
        rate_hz = 3
        # Start the ROS node
        rospy.init_node("test_frontiers")
        pub_depth = rospy.Publisher("test_depth", Image, queue_size=10)
        pub_pose = rospy.Publisher("test_pose", PoseStamped, queue_size=100)
        rate = rospy.Rate(rate_hz)
        rospy.loginfo("Publishing test depth")
        while not rospy.is_shutdown():
            depth = generate_test_image(width, height, near_wall_m, far_wall_m)
            pub_depth.publish(depth)
            pub_pose.publish(generate_test_pose())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

