// SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __EIGEN_ROS_CONVERSIONS_IMPL_HPP
#define __EIGEN_ROS_CONVERSIONS_IMPL_HPP



namespace se {
  inline geometry_msgs::Point make_point(float x, float y, float z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  inline geometry_msgs::Point make_point(float x) {
    return make_point(x, x, x);
  }



  inline geometry_msgs::Point eigen_to_point(const Eigen::Vector3f& p) {
    geometry_msgs::Point pp;
    pp.x = p.x();
    pp.y = p.y();
    pp.z = p.z();
    return pp;
  }



  inline geometry_msgs::Vector3 make_vector3(float x, float y, float z) {
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  inline geometry_msgs::Vector3 make_vector3(float x) {
    return make_vector3(x, x, x);
  }



  inline geometry_msgs::Vector3 eigen_to_vector3(const Eigen::Vector3f& p) {
    geometry_msgs::Vector3 v;
    v.x = p.x();
    v.y = p.y();
    v.z = p.z();
    return v;
  }



  inline geometry_msgs::Quaternion make_quaternion(float x, float y, float z, float w) {
    geometry_msgs::Quaternion qq;
    qq.x = x;
    qq.y = y;
    qq.z = z;
    qq.w = w;
    return qq;
  }



  inline geometry_msgs::Quaternion eigen_to_quaternion(const Eigen::Quaternionf& q) {
    geometry_msgs::Quaternion qq;
    qq.x = q.x();
    qq.y = q.y();
    qq.z = q.z();
    qq.w = q.w();
    return qq;
  }



  inline geometry_msgs::Pose eigen_to_pose(const Eigen::Matrix4f& T) {
    geometry_msgs::Pose p;
    p.position = eigen_to_point(T.topRightCorner<3,1>());
    p.orientation = eigen_to_quaternion(Eigen::Quaternionf(T.topLeftCorner<3,3>()));
    return p;
  }



  inline geometry_msgs::Transform eigen_to_transform(const Eigen::Matrix4f& T) {
    geometry_msgs::Transform t;
    t.translation = eigen_to_vector3(T.topRightCorner<3,1>());
    t.rotation = eigen_to_quaternion(Eigen::Quaternionf(T.topLeftCorner<3,3>()));
    return t;
  }



  inline std_msgs::ColorRGBA make_color(float r, float g, float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }

  inline std_msgs::ColorRGBA eigen_to_color(const Eigen::Vector4f& color) {
    std_msgs::ColorRGBA c;
    c.r = color.x();
    c.g = color.y();
    c.b = color.z();
    c.a = color.w();
    return c;
  }
} // namespace se

#endif

