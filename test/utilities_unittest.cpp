// SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>

#include "supereight_ros/utilities.hpp"



class UtilitiesTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      // Initialize the TransformStamped messages
      tf1_msg.header.seq = 0;
      tf1_msg.header.stamp.sec = 0;
      tf1_msg.header.stamp.nsec = 0;
      tf1_msg.header.frame_id = "";
      tf1_msg.child_frame_id = "world";
      tf1_msg.transform.translation.x = 1.0;
      tf1_msg.transform.translation.y = 2.0;
      tf1_msg.transform.translation.z = 3.0;
      tf1_msg.transform.rotation.x = 0.0;
      tf1_msg.transform.rotation.y = 0.0;
      tf1_msg.transform.rotation.z = 0.0;
      tf1_msg.transform.rotation.w = 1.0;
      // tf2_msg is translated by [1 0 0] and rotated by 90 degrees around the
      // z axis compared to tf1_msg
      tf2_msg.header.seq = 1;
      tf2_msg.header.stamp.sec = 3;
      tf2_msg.header.stamp.nsec = 0;
      tf2_msg.header.frame_id = "";
      tf2_msg.child_frame_id = "world";
      tf2_msg.transform.translation = tf1_msg.transform.translation;
      tf2_msg.transform.translation.x += 1.0;
      tf2_msg.transform.rotation.x = 0.0;
      tf2_msg.transform.rotation.y = 0.0;
      tf2_msg.transform.rotation.z = std::sqrt(2.0) / 2.0;
      tf2_msg.transform.rotation.w = std::sqrt(2.0) / 2.0;
      // Initialize the PoseStamped message
      pose_msg.header.seq = 0;
      pose_msg.header.stamp.sec = 0;
      pose_msg.header.stamp.nsec = 0;
      pose_msg.header.frame_id = "";
      pose_msg.pose.position.x = 1.0;
      pose_msg.pose.position.y = 2.0;
      pose_msg.pose.position.z = 3.0;
      pose_msg.pose.orientation.x = 0.0;
      pose_msg.pose.orientation.y = 0.0;
      pose_msg.pose.orientation.z = 0.0;
      pose_msg.pose.orientation.w = 1.0;
      // Initialize the Eigen pose/transform
      tf1_eigen = Eigen::Matrix4f::Identity();
      tf1_eigen(0,3) = 1.f;
      tf1_eigen(1,3) = 2.f;
      tf1_eigen(2,3) = 3.f;
      // Initialize the Eigen interpolated pose. It should be translated by
      // [0.5 0 0] and rotated by 45 degrees around the z axis compared to
      // tf1_msg
      desired_interpolated_pose = Eigen::Matrix4f::Identity();
      // Position
      desired_interpolated_pose(0,3) = 1.5f;
      desired_interpolated_pose(1,3) = 2.f;
      desired_interpolated_pose(2,3) = 3.f;
      // Orientation
      desired_interpolated_pose(0,0) =  std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(0,1) = -std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(1,0) =  std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(1,1) =  std::sqrt(2.0) / 2.0;
    }

  geometry_msgs::TransformStamped tf1_msg;
  geometry_msgs::TransformStamped tf2_msg;
  geometry_msgs::PoseStamped pose_msg;
  Eigen::Matrix4f tf1_eigen;
  const Eigen::Matrix4f& pose_eigen = tf1_eigen;
  Eigen::Matrix4f desired_interpolated_pose;
};



//TEST_F(UtilitiesTest, toSupereightDepth) {
//  ASSERT_TRUE(false);
//}



//TEST_F(UtilitiesTest, toSupereightRGB) {
//  ASSERT_TRUE(false);
//}



//TEST_F(UtilitiesTest, RGBAToMsg) {
//  ASSERT_TRUE(false);
//}



TEST_F(UtilitiesTest, transformToEigen) {
  const Eigen::Matrix4f tf = se::transform_msg_to_eigen(tf1_msg);
  ASSERT_TRUE(tf.isApprox(tf1_eigen));
}



TEST_F(UtilitiesTest, poseToEigen) {
  const Eigen::Matrix4f pose = se::pose_msg_to_eigen(pose_msg);
  ASSERT_TRUE(pose.isApprox(pose_eigen));
}



TEST_F(UtilitiesTest, InterpolatePose) {
  const double ts = ros::Time({1, 500000000}).toSec();
  const Eigen::Matrix4f pose = se::interpolate_pose(tf1_msg, tf2_msg, ts);
  ASSERT_TRUE(pose.isApprox(desired_interpolated_pose));
}



TEST_F(UtilitiesTest, InterpolatePoseSame) {
  const Eigen::Matrix4f pose = se::interpolate_pose(tf1_msg, tf1_msg, -1.0);
  ASSERT_TRUE(pose.isApprox(tf1_eigen));
}



TEST_F(UtilitiesTest, InterpolatePoseWrongOrder) {
  const double ts = ros::Time({1, 500000000}).toSec();
  const Eigen::Matrix4f pose = se::interpolate_pose(tf2_msg, tf1_msg, ts);
  ASSERT_TRUE(pose.isApprox(desired_interpolated_pose));
}



//TEST_F(UtilitiesTest, GetSurroundingPoses) {
//  ASSERT_TRUE(false);
//}



//TEST_F(UtilitiesTest, GetClosestImage) {
//  ASSERT_TRUE(false);
//}

