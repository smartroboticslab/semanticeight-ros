// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/utilities.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include "se/image_utils.hpp"



namespace se {
  void to_supereight_depth(const sensor_msgs::ImageConstPtr& input_depth,
                           const float                       far_plane,
                           float*                            output_depth) {

    // Values equal to the camera far plane should be ignored because Gazebo
    // camera plugins return the value of the far plane for rays that don't hit
    // anything.

    // The input depth is in float meters
    if (input_depth->encoding == "32FC1" && !input_depth->is_bigendian) {
      const float* d = reinterpret_cast<const float*>(input_depth->data.data());
      memcpy(output_depth, d, sizeof(float) * input_depth->width * input_depth->height);

    // The depth is in uint16_t millimeters
    } else if (((input_depth->encoding == "16UC1") || (input_depth->encoding == "mono16"))
         && !input_depth->is_bigendian) {
      const uint16_t* d = reinterpret_cast<const uint16_t*>(input_depth->data.data());
#pragma omp parallel for
      for (size_t i = 0; i < input_depth->width * input_depth->height; ++i) {
        const float depth_m = d[i] / 1000.0f;
        if (depth_m >= far_plane) {
          output_depth[i] = 0.0f;
        } else {
          output_depth[i] = depth_m;
        }
      }

    // Invalid format.
    } else {
      ROS_FATAL("Invalid input depth format %s, expected little endian mono16, 16UC1 or 32FC1",
          input_depth->encoding.c_str());
      abort();
    }
  }



  void to_supereight_RGB(const sensor_msgs::ImageConstPtr& input_color,
                         uint32_t*                         output_rgba) {

    // Just copy the image data since this is already the correct format.
    if ((input_color->encoding == "8UC4") || (input_color->encoding == "rgba8")) {
      std::memcpy(output_rgba, input_color->data.data(), input_color->data.size());

    // Add the alpha channel.
    } else if ((input_color->encoding == "8UC3") || (input_color->encoding == "rgb8")) {
      // Iterate over all pixels.
#pragma omp parallel for
      for (size_t i = 0; i < input_color->width * input_color->height; ++i) {
        const uint8_t r = input_color->data[3*i + 0];
        const uint8_t g = input_color->data[3*i + 1];
        const uint8_t b = input_color->data[3*i + 2];
        output_rgba[i] = se::pack_rgba(r, g, b, 0xFF);
      }

    // Duplicate the grayscale channel and add alpha.
    } else if ((input_color->encoding == "8UC1") || (input_color->encoding == "mono8")) {
      // Iterate over all pixels.
#pragma omp parallel for
      for (size_t i = 0; i < input_color->width * input_color->height; ++i) {
        const uint8_t v = input_color->data[i];
        output_rgba[i] = se::pack_rgba(v, v, v, 0xFF);
      }
      ROS_WARN_ONCE("Using a %s image as RGB, semantics might not work well", input_color->encoding.c_str());

    // Invalid format.
    } else {
      ROS_FATAL("Invalid input RGB format %s, expected rgb8, 8UC3, rgba8 or 8UC4",
          input_color->encoding.c_str());
      abort();
    }
  }



  se::SegmentationResult to_supereight_segmentation(const sensor_msgs::ImageConstPtr& input_class,
                                                    const sensor_msgs::ImageConstPtr& input_instance) {
    // Ensure the image types are correct
    if ((input_class->encoding != "8UC1") && (input_class->encoding != "mono8")) {
      ROS_FATAL("Invalid input class format %s, expected mono8 or 8UC1",
          input_class->encoding.c_str());
      abort();
    }
    if ((input_instance->encoding != "16UC1") && (input_instance->encoding != "mono16")) {
      ROS_FATAL("Invalid input class format %s, expected mono16 or 16UC1",
          input_instance->encoding.c_str());
      abort();
    }
    const uint32_t w = input_class->width;
    const uint32_t h = input_class->height;
    // Convert to OpenCV images
    cv_bridge::CvImageConstPtr class_img = cv_bridge::toCvShare(input_class);
    cv_bridge::CvImageConstPtr instance_img = cv_bridge::toCvShare(input_instance);
    // Create a map from instance IDs to segmentations
    std::map<uint16_t, se::InstanceSegmentation> m;
    // Iterate over each pixel in the instance image
    for (int y = 0; y < instance_img->image.rows; ++y) {
      for (int x = 0; x < instance_img->image.cols; ++x) {
        const uint16_t instance_id = instance_img->image.at<uint16_t>(y, x);
        // Test if we've seen this ID before
        if (m.find(instance_id) == m.end()) {
          // Get the corresponding class ID
          const uint8_t class_id = class_img->image.at<uint8_t>(y, x);
          // Initialize an InstanceSegmentation for this ID
          m[instance_id] = se::InstanceSegmentation(instance_id, class_id, cv::Mat::zeros(h, w, se::mask_t));
        }
        // Set the instance mask value
        m[instance_id].instance_mask.at<se::mask_elem_t>(y, x) = 255;
      }
    }
    // Convert the std::map to an se::SegmentationResult
    se::SegmentationResult segm (w, h);
    segm.object_instances.reserve(m.size());
    for (const auto& instance : m) {
      segm.object_instances.push_back(instance.second);
    }
    return segm;
  }



  sensor_msgs::Image RGBA_to_msg(const uint32_t*         image_data,
                                 const Eigen::Vector2i&  image_res,
                                 const std_msgs::Header& header) {

    const size_t num_bytes = image_res.prod() * sizeof(uint32_t);
    sensor_msgs::Image image;

    image.header       = header;
    image.height       = image_res.y();
    image.width        = image_res.x();
    image.encoding     = "rgba8";
    image.is_bigendian = 0;
    image.step         = sizeof(uint32_t) * image_res.x();
    image.data         = std::vector<uint8_t>(num_bytes);
    memcpy(image.data.data(), image_data, num_bytes);

    return image;
  }



  nav_msgs::Path path_to_path_msg(const se::Path&         path_WB,
                                  const std_msgs::Header& header) {
    nav_msgs::Path path_msg;
    path_msg.header = header;
    path_msg.poses.resize(path_WB.size());
    for (size_t i = 0; i < path_WB.size(); ++i) {
      path_msg.poses[i].header = header;
      const Eigen::Matrix4f T_WB = path_WB[i];
      const Eigen::Vector3f t_WB = T_WB.topRightCorner<3,1>();
      const Eigen::Quaternionf q_WB (T_WB.topLeftCorner<3,3>());
      path_msg.poses[i].pose.position.x = t_WB.x();
      path_msg.poses[i].pose.position.y = t_WB.y();
      path_msg.poses[i].pose.position.z = t_WB.z();
      path_msg.poses[i].pose.orientation.x = q_WB.x();
      path_msg.poses[i].pose.orientation.y = q_WB.y();
      path_msg.poses[i].pose.orientation.z = q_WB.z();
      path_msg.poses[i].pose.orientation.w = q_WB.w();
    }
    return path_msg;
  }


  trajectory_msgs::MultiDOFJointTrajectory path_to_traj_msg(const se::Path&         path_WB,
                                                            const std_msgs::Header& header) {
    trajectory_msgs::MultiDOFJointTrajectory path_msg;
    path_msg.header = header;
    path_msg.points.resize(path_WB.size());
    for (size_t i = 0; i < path_WB.size(); ++i) {
      const Eigen::Matrix4f T_WB = path_WB[i];
      const Eigen::Vector3f t_WB = T_WB.topRightCorner<3,1>();
      const Eigen::Quaterniond q_WB((T_WB.topLeftCorner<3,3>()).cast<double>());

      mav_msgs::EigenTrajectoryPoint point;
      point.position_W.x() = t_WB.x();
      point.position_W.y() = t_WB.y();
      point.position_W.z() = t_WB.z();
      // Take the constant tracking error of the controller into account
      point.position_W.z() += 0.2;
      tf::Quaternion q_tf_WB;
      tf::quaternionEigenToTF(q_WB, q_tf_WB);
      double yaw = tf::getYaw(q_tf_WB);
      point.setFromYaw(yaw);

      trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(point, &point_msg);
      path_msg.points.push_back(point_msg);
    }
    return path_msg;
  }


  Eigen::Matrix4f interpolate_pose(
      const geometry_msgs::TransformStamped& prev_pose,
      const geometry_msgs::TransformStamped& next_pose,
      const double                           query_timestamp) {

    // Return the prev_pose if both poses have the exact same timestamp
    if    ((prev_pose.header.stamp.sec  == next_pose.header.stamp.sec)
        && (prev_pose.header.stamp.nsec == next_pose.header.stamp.nsec)) {
      return transform_msg_to_eigen(prev_pose);
    }

    // Convert from ROS to Eigen
    const Eigen::Vector3f prev_translation(
        prev_pose.transform.translation.x,
        prev_pose.transform.translation.y,
        prev_pose.transform.translation.z);
    const Eigen::Vector3f next_translation(
        next_pose.transform.translation.x,
        next_pose.transform.translation.y,
        next_pose.transform.translation.z);

    const Eigen::Quaternionf prev_rotation(
        prev_pose.transform.rotation.w,
        prev_pose.transform.rotation.x,
        prev_pose.transform.rotation.y,
        prev_pose.transform.rotation.z);
    const Eigen::Quaternionf next_rotation(
        next_pose.transform.rotation.w,
        next_pose.transform.rotation.x,
        next_pose.transform.rotation.y,
        next_pose.transform.rotation.z);

    // Interpolate translation and rotation separately
    const double prev_timestamp = ros::Time(prev_pose.header.stamp).toSec();
    const double next_timestamp = ros::Time(next_pose.header.stamp).toSec();
    const float alpha
        = (query_timestamp - prev_timestamp) / (next_timestamp - prev_timestamp);
    const Eigen::Vector3f inter_translation
        = (1.f - alpha) * prev_translation + alpha * next_translation;
    const Eigen::Quaternionf inter_rotation
        = prev_rotation.slerp(alpha, next_rotation);

    // Combine into homogeneous transform
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = inter_translation;
    pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

    return pose;
  }



  InterpResult get_surrounding_poses(
      const boost::circular_buffer<geometry_msgs::TransformStamped>& buffer,
      const double                                                   query_timestamp,
      geometry_msgs::TransformStamped&                               prev_pose,
      geometry_msgs::TransformStamped&                               next_pose) {

    for (size_t i = 0; i < buffer.size(); ++i) {
      const double pose_timestamp = ros::Time(buffer[i].header.stamp).toSec();

      if (pose_timestamp == query_timestamp) {
        // Found an exact timestamp.
        prev_pose = buffer[i];
        next_pose = buffer[i];
        return InterpResult::ok;

      } else if (pose_timestamp > query_timestamp) {
        if (i > 0) {
          // Found previous and next poses.
          prev_pose = buffer[i - 1];
          next_pose = buffer[i];
          return InterpResult::ok;
        } else {
          // The query_timestamp is smaller than all the pose timestamps.
          return InterpResult::query_smaller;
        }
      }
    }

    // The query_timestamp is greater than all the pose timestamps.
    return InterpResult::query_greater;
  }



  Eigen::Matrix4f transform_msg_to_eigen(const geometry_msgs::TransformStamped& tf_msg) {
    const Eigen::Vector3f translation (
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z);
    const Eigen::Quaternionf rotation (
        tf_msg.transform.rotation.w,
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z);
    // Combine into homogeneous transform
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.block<3, 1>(0, 3) = translation;
    tf.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return tf;
  }



  Eigen::Matrix4f pose_msg_to_eigen(const geometry_msgs::PoseStamped& pose_msg) {
    const Eigen::Vector3f position (
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z);
    const Eigen::Quaternionf orientation (
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z);
    // Combine into homogeneous transform
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = position;
    pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    return pose;
  }

} // namespace se

