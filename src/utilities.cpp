//
// Created by anna on 16/05/19.
//

#include "supereight_ros/utilities.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

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
        if (std::isnan(depth_mm) || (depth_mm > (1 << 16) - 1)) {
          output_depth[i] = 0;
        } else {
          output_depth[i] = static_cast<uint16_t>(depth_mm);
        }
      }

    } else {
      ROS_FATAL_STREAM("Invalid input depth format: " << input_depth.encoding);
      abort();
    }
  }



  void createImageMsg(const sensor_msgs::ImageConstPtr& old_image_msg,
                      sensor_msgs::ImagePtr&            new_image_msg) {

    new_image_msg->header       = old_image_msg->header;
    new_image_msg->height       = old_image_msg->height;
    new_image_msg->width        = old_image_msg->width;
    new_image_msg->encoding     = old_image_msg->encoding;
    new_image_msg->is_bigendian = old_image_msg->is_bigendian;
    new_image_msg->step         = sizeof(float) * new_image_msg->width; // TODO fix this hack.
  }



  void createImageMsg(const supereight_ros::ImagePose::ConstPtr& old_image_msg,
                      sensor_msgs::ImagePtr&                     new_image_msg,
                      Eigen::Vector2i&                           image_size) {

    new_image_msg->header = old_image_msg->image.header;
    new_image_msg->height = image_size.y();
    new_image_msg->width  = image_size.x();
    new_image_msg->step   = sizeof(float) * image_size.x(); //sizeof(float)=4 // TODO fix this hack.
    new_image_msg->data   = std::vector<uint8_t>(image_size.x() * image_size.y() * sizeof(float));
  }



  Eigen::Matrix4f swapAxes(const Eigen::Matrix4f& input) {
    Eigen::Matrix4f output = input;
    output.block<3, 1>(0, 0) = -input.block<3, 1>(0, 1);
    output.block<3, 1>(0, 1) = -input.block<3, 1>(0, 2);
    output.block<3, 1>(0, 2) =  input.block<3, 1>(0, 0);
    return output;
  }



  void createTestImage(const sensor_msgs::ImageConstPtr& disp_msg,
                       sensor_msgs::ImagePtr&            depth_msg) {

    const int row_step = depth_msg->step / sizeof(float);
    float *depth_data = reinterpret_cast<float *>(&depth_msg->data[0]);
    for (int v = 0; v < (int) depth_msg->height; ++v) {
      for (int u = 0; u < (int) depth_msg->width; ++u) {
        *depth_data = 3.0;
        ++depth_data;
      }
    }
    int ncol = 300;
    float *depth_data2 = reinterpret_cast<float *>(&depth_msg->data[ncol + 80 * row_step]);
    for (int h = 80; h < (int) depth_msg->height; h++){
      for (int w = ncol; w < ncol + 100 ;w++){
        *depth_data2= 5.8;
        ++depth_data2;
      }
      depth_data2 = depth_data2 + row_step- 100;
    }

    ncol = 900;
    float *depth_data3 = reinterpret_cast<float *>(&depth_msg->data[ncol + 280 * row_step]);
    for(int h = 280 ; h < (int) depth_msg->height ; h ++){
      for (int w = ncol; w < ncol + 150; w++){
        *depth_data3 = 1.f;
        ++depth_data3;
      }
      depth_data3=depth_data3 + row_step - 150;
    }
    ncol = 1600;
    float *depth_data4 = reinterpret_cast<float *>(&depth_msg->data[ncol + 380 * row_step]);
    for(int h = 380 ; h < (int) depth_msg->height ; h ++){
      for (int w = ncol; w < ncol + 150; w++){
        *depth_data4 = 0.f;
        ++depth_data4;
      }
      depth_data4=depth_data4 + row_step - 150;
    }
  }



  float compute_alpha(const int64_t prev_timestamp,
                      const int64_t query_timestamp,
                      const int64_t next_timestamp) {

    return static_cast<float>(query_timestamp - prev_timestamp)
         / static_cast<float>(next_timestamp - prev_timestamp);
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
      const int64_t                          query_timestamp) {

    const float alpha = compute_alpha(
        ros::Time(prev_pose.header.stamp).toNSec(),
        query_timestamp,
        ros::Time(next_pose.header.stamp).toNSec());

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

} // namespace se

