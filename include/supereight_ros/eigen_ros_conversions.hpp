// SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __EIGEN_ROS_CONVERSIONS_HPP
#define __EIGEN_ROS_CONVERSIONS_HPP

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/ColorRGBA.h>



namespace se {
// Point
inline geometry_msgs::Point make_point(float x, float y, float z);
inline geometry_msgs::Point make_point(float x = 0.0f);
inline geometry_msgs::Point eigen_to_point(const Eigen::Vector3f& p);

// Vector3
inline geometry_msgs::Vector3 make_vector3(float x, float y, float z);
inline geometry_msgs::Vector3 make_vector3(float x = 0.0f);
inline geometry_msgs::Vector3 eigen_to_vector3(const Eigen::Vector3f& p);

// Quaternion
inline geometry_msgs::Quaternion
make_quaternion(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f);
inline geometry_msgs::Quaternion eigen_to_quaternion(const Eigen::Quaternionf& q);

inline geometry_msgs::Pose eigen_to_pose(const Eigen::Matrix4f& T);

inline geometry_msgs::Transform eigen_to_transform(const Eigen::Matrix4f& T);

inline std_msgs::ColorRGBA make_color(float r, float g, float b, float a);

inline std_msgs::ColorRGBA eigen_to_color(const Eigen::Vector4f& color);
} // namespace se

#include "supereight_ros/eigen_ros_conversions_impl.hpp"

#endif
