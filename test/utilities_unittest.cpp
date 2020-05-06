// SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

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
      tf2_msg.header.frame_id = tf1_msg.header.frame_id;
      tf2_msg.child_frame_id = tf1_msg.child_frame_id;
      tf2_msg.transform.translation = tf1_msg.transform.translation;
      tf2_msg.transform.translation.x += 1.0;
      tf2_msg.transform.rotation.x = 0.0;
      tf2_msg.transform.rotation.y = 0.0;
      tf2_msg.transform.rotation.z = std::sqrt(2.0) / 2.0;
      tf2_msg.transform.rotation.w = std::sqrt(2.0) / 2.0;
      // Initialize the PoseStamped message (same data as tf1_msg)
      pose_msg.header = tf1_msg.header;
      pose_msg.pose.position.x = tf1_msg.transform.translation.x;
      pose_msg.pose.position.y = tf1_msg.transform.translation.y;
      pose_msg.pose.position.z = tf1_msg.transform.translation.z;
      pose_msg.pose.orientation.x = tf1_msg.transform.rotation.x;
      pose_msg.pose.orientation.y = tf1_msg.transform.rotation.y;
      pose_msg.pose.orientation.z = tf1_msg.transform.rotation.z;
      pose_msg.pose.orientation.w = tf1_msg.transform.rotation.w;
      // Initialize the Eigen pose/transform (same data as tf1_msg)
      tf1_eigen = Eigen::Matrix4f::Identity();
      tf1_eigen(0,3) = tf1_msg.transform.translation.x;
      tf1_eigen(1,3) = tf1_msg.transform.translation.y;
      tf1_eigen(2,3) = tf1_msg.transform.translation.z;
      // Initialize the Eigen interpolated pose. It should be translated by
      // [0.5 0 0] and rotated by 45 degrees around the z axis compared to
      // tf1_msg
      desired_interpolated_pose = Eigen::Matrix4f::Identity();
      // Position
      desired_interpolated_pose(0,3) = tf1_msg.transform.translation.x + 0.5f;
      desired_interpolated_pose(1,3) = tf1_msg.transform.translation.y;
      desired_interpolated_pose(2,3) = tf1_msg.transform.translation.z;
      // Orientation
      desired_interpolated_pose(0,0) =  std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(0,1) = -std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(1,0) =  std::sqrt(2.0) / 2.0;
      desired_interpolated_pose(1,1) =  std::sqrt(2.0) / 2.0;

      // Ensure the depth image parameters won't cause an overflow of a uint16_t
      ASSERT_LT(depth_scale * (w*h - 1), UINT16_MAX);
      // Initialize the depth images
      depth_uint16_msg = init_image_msg(tf1_msg.header, w, h, "mono16");
      depth_float_msg = init_image_msg(tf1_msg.header, w, h, "32FC1");
      desired_se_depth = std::unique_ptr<uint16_t>(new uint16_t[w * h]);
      // Pointers to access the uint8_t vectors pixel-by-pixel
      uint16_t* depth_uint16_data
          = reinterpret_cast<uint16_t*>(depth_uint16_msg.data.data());
      float* depth_float_data
          = reinterpret_cast<float*>(depth_float_msg.data.data());
      // Set the image data
      for (size_t p = 0; p < w * h - 3; ++p) {
        desired_se_depth.get()[p] = depth_scale * p;
        depth_uint16_data[p] = depth_scale * p;
        depth_float_data[p] = depth_scale / 1000.f * p;
      }
      // Use the last pixels to test special float values
      // NaN
      desired_se_depth.get()[w * h - 3] = 0;
      depth_uint16_data[w * h - 3]      = 0;
      depth_float_data[w * h - 3]       = std::nanf("");
      // Less than 1 mm, the resolution of uint16_t
      desired_se_depth.get()[w * h - 2] = 0;
      depth_uint16_data[w * h - 2]      = 0;
      depth_float_data[w * h - 2]       = 0.00001f;
      // Will overflow uint16_t
      desired_se_depth.get()[w * h - 1] = 0;
      depth_uint16_data[w * h - 1]      = 0;
      depth_float_data[w * h - 1]       = 70.f;

      // Ensure the RGB image parameters won't cause an overflow of a uint8_t
      ASSERT_LT((w*h - 1), UINT8_MAX);
      // Initialize the RGB/RGBA images
      rgb_msg = init_image_msg(tf1_msg.header, w, h, "rgb8");
      rgba_msg = init_image_msg(tf1_msg.header, w, h, "rgba8");
      desired_se_rgb = std::unique_ptr<uint8_t>(new uint8_t[w * h * 3]);
      // Set the image data
      for (size_t p = 0; p < w * h * 3; ++p) {
        desired_se_rgb.get()[p] = p / 3;
        rgb_msg.data[p] = p / 3;
        // Skip the alpha channel of the RGBA image
        const size_t rgba_idx = p + (p / 3);
        rgba_msg.data[rgba_idx] = p / 3;
      }
    }



    sensor_msgs::Image init_image_msg(const std_msgs::Header& header,
                                      size_t                  width,
                                      size_t                  height,
                                      const std::string&      encoding) {
      // Compute pixel size depending on encoding
      size_t pixel_size = 0;
      if ((encoding == "mono16") || (encoding == "16UC1")) {
        pixel_size = sizeof(uint16_t);
      } else if (encoding == "32FC1") {
        pixel_size = sizeof(float);
      } else if ((encoding == "rgb8") || (encoding == "8UC3")) {
        pixel_size = 3 * sizeof(uint8_t);
      } else if ((encoding == "rgba8") || (encoding == "8UC4")) {
        pixel_size = sizeof(uint32_t);
      }
      sensor_msgs::Image img;
      img.header = header;
      img.width = width;
      img.height = height;
      img.encoding = encoding;
      img.is_bigendian = 0;
      img.step = width * pixel_size;
      img.data.resize(width * height * pixel_size);
      return img;
    }



    // Transform/Pose
    geometry_msgs::TransformStamped tf1_msg;
    geometry_msgs::TransformStamped tf2_msg;
    geometry_msgs::PoseStamped pose_msg;
    Eigen::Matrix4f tf1_eigen;
    const Eigen::Matrix4f& pose_eigen = tf1_eigen;
    Eigen::Matrix4f desired_interpolated_pose;

    // Depth
    const size_t w = 16;
    const size_t h = 8;
    const int depth_scale = 100;
    sensor_msgs::Image depth_uint16_msg;
    sensor_msgs::Image depth_float_msg;
    std::unique_ptr<uint16_t> desired_se_depth;
    const size_t desired_se_depth_size = w * h * sizeof(uint16_t);

    // RGB/RGBA
    sensor_msgs::Image rgb_msg;
    sensor_msgs::Image rgba_msg;
    std::unique_ptr<uint8_t> desired_se_rgb;
    const size_t desired_se_rgb_size = w * h * 3 * sizeof(uint8_t);
};



TEST_F(UtilitiesTest, uint16ToSupereightDepth) {
  std::unique_ptr<uint16_t> converted_depth (new uint16_t[w * h]);
  se::to_supereight_depth(depth_uint16_msg, converted_depth.get());
  ASSERT_FALSE(std::memcmp(converted_depth.get(), desired_se_depth.get(),
        desired_se_depth_size));
}



TEST_F(UtilitiesTest, floatToSupereightDepth) {
  std::unique_ptr<uint16_t> converted_depth (new uint16_t[w * h]);
  se::to_supereight_depth(depth_float_msg, converted_depth.get());
  ASSERT_FALSE(std::memcmp(converted_depth.get(), desired_se_depth.get(),
        desired_se_depth_size));
}



TEST_F(UtilitiesTest, RGBToSupereightRGB) {
  std::unique_ptr<uint8_t> converted_rgb (new uint8_t[w * h * 3]);
  se::to_supereight_RGB(rgb_msg, converted_rgb.get());
  ASSERT_FALSE(std::memcmp(converted_rgb.get(), desired_se_rgb.get(),
        desired_se_rgb_size));
}



TEST_F(UtilitiesTest, RGBAToSupereightRGB) {
  std::unique_ptr<uint8_t> converted_rgb (new uint8_t[w * h * 3]);
  se::to_supereight_RGB(rgba_msg, converted_rgb.get());
  ASSERT_FALSE(std::memcmp(converted_rgb.get(), desired_se_rgb.get(),
        desired_se_rgb_size));
}



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

