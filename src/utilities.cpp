/*
 * SPDX-FileCopyrightText: 2019 Anna Dai
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "supereight_ros/utilities.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

#include <ros/ros.h>



namespace se {
  void to_supereight_depth(const sensor_msgs::Image& input_depth,
                           uint16_t*                 output_depth) {

    if (input_depth.encoding == "16UC1") {
      // Just copy the image data since this is already the correct format.
      const size_t image_size_bytes = input_depth.height * input_depth.step;
      std::memcpy(output_depth, input_depth.data.data(), image_size_bytes);

    } else if (input_depth.encoding == "32FC1") {
      // The depth is in float meters, convert to uint16_t millimeters.
      const size_t image_size_pixels = input_depth.width * input_depth.height;
      // Cast the image data as a float pointer so that operator[] can be used
      // to get the value of each pixel.
      const float* input_ptr
          = reinterpret_cast<const float*>(input_depth.data.data());
#pragma omp parallel for
      for (size_t i = 0; i < image_size_pixels; ++i) {
        const float depth_mm = 1000.f * input_ptr[i];
        // Test whether the depth value is NaN or if it would cause an overflow
        // in a uint16_t. In that case store an invalid depth value.
        if (std::isnan(depth_mm) || (depth_mm > UINT16_MAX)) {
          output_depth[i] = 0;
        } else {
          output_depth[i] = static_cast<uint16_t>(depth_mm);
        }
      }
    } else {
      ROS_FATAL("Invalid input depth format %s, expected 16UC1 or 32FC1",
          input_depth.encoding.c_str());
      abort();
    }
  }



  sensor_msgs::Image msg_from_RGB_image(
      const uint32_t*                   image_data,
      const Eigen::Vector2i&            image_size,
      const sensor_msgs::ImageConstPtr& header_source) {

    const size_t num_bytes = image_size.x() * image_size.y() * sizeof(uint32_t);
    sensor_msgs::Image image;

    image.header       = header_source->header;
    image.height       = image_size.y();
    image.width        = image_size.x();
    image.encoding     = "rgba8";
    image.is_bigendian = 0;
    image.step         = sizeof(uint32_t) * image_size.x();
    image.data         = std::vector<uint8_t>(num_bytes);
    memcpy(image.data.data(), image_data, num_bytes);

    return image;
  }



  float compute_alpha(const double prev_timestamp,
                      const double query_timestamp,
                      const double next_timestamp) {

    return (query_timestamp - prev_timestamp) / (next_timestamp - prev_timestamp);
  }



  Eigen::Vector3f interpolate_position(const Eigen::Vector3f& prev_pos,
                                       const Eigen::Vector3f& next_pos,
                                       const float            alpha) {

    return prev_pos + alpha * (next_pos - prev_pos);
  }



  Eigen::Quaternionf interpolate_orientation(
      const Eigen::Quaternionf& prev_orientation,
      const Eigen::Quaternionf& next_orientation,
      const float               alpha) {

    return prev_orientation.slerp(alpha, next_orientation);
  }



  Eigen::Matrix4f interpolate_pose(
      const geometry_msgs::TransformStamped& prev_pose,
      const geometry_msgs::TransformStamped& next_pose,
      const double                           query_timestamp) {

    const float alpha = compute_alpha(
        ros::Time(prev_pose.header.stamp).toSec(),
        query_timestamp,
        ros::Time(next_pose.header.stamp).toSec());

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

    const Eigen::Vector3f inter_translation
        = interpolate_position(prev_translation, next_translation, alpha);
    const Eigen::Quaternionf inter_rotation
        = interpolate_orientation(prev_rotation, next_rotation, alpha);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = inter_translation;
    pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

    return pose;
  }



  void print_timings(
      const std::vector<std::chrono::time_point<std::chrono::steady_clock> >& timings,
      const std::vector<std::string>& labels) {

    for (size_t i = 0; i < timings.size() - 1; ++i) {
      const double delta_t
          = std::chrono::duration<double>(timings[i+1] - timings[i]).count();
      ROS_INFO("%-25s %.5f s", labels[i].c_str(), delta_t);
    }
    const double total_t
        = std::chrono::duration<double>(timings[timings.size()-1] - timings[0]).count();
    ROS_INFO("Total                     %.5f s", total_t);
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



  bool get_closest_image(
      const boost::circular_buffer<sensor_msgs::ImageConstPtr>& buffer,
      const double                                              query_timestamp,
      const double                                              threshold,
      sensor_msgs::ImageConstPtr&                               closest_image) {

    // Find the minimum time difference.
    double min_time_diff = std::numeric_limits<double>::infinity();
    sensor_msgs::ImageConstPtr min_img;
    for (const auto img : buffer) {
      const double img_timestamp = ros::Time(img->header.stamp).toSec();
      const double time_diff = std::fabs(img_timestamp - query_timestamp);

      if (time_diff <= min_time_diff) {
        min_time_diff = time_diff;
        min_img = img;
      } else {
        // The time difference started increasing which means the minimum has
        // been found, finish early.
        break;
      }
    }

    if (min_time_diff <= threshold) {
      closest_image = min_img;
      return true;
    } else {
      return false;
    }
  }

} // namespace se

