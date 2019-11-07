//
// Created by anna on 16/05/19.
//

#ifndef __UTILITIES_HPP
#define __UTILITIES_HPP

#include <cstdint>

#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

#include "supereight_ros/ImagePose.h"



namespace se {
  /**
   * @brief Convert the input depth image into the format required by
   * supereight and copy it into the output depth buffer.
   */
  void to_supereight_depth(const sensor_msgs::Image& input_depth,
                           uint16_t*                 output_depth);



  /**
   * @brief create new image message with all attributes
   * @param old_image_msg
   * @param new_image_msg
   */
  void createImageMsg(const sensor_msgs::ImageConstPtr& old_image_msg,
                      sensor_msgs::ImagePtr&            new_image_msg);



  /**
   * @brief create new image message with all attributes
   * @param old_image_msg
   * @param new_image_msg
   * @param image_size
   */
  void createImageMsg(const supereight_ros::ImagePose::ConstPtr& old_image_msg,
                      sensor_msgs::ImagePtr&                     new_image_msg,
                      Eigen::Vector2i&                           image_size);



  /**
   * @brief swaps the rotation axis to convert the world frame to octree frame
   * @param[in] camera pose in world frame
   * @return rotated pose
   */
  Eigen::Matrix4f swapAxes(const Eigen::Matrix4f& input);



  void createTestImage(const sensor_msgs::ImageConstPtr& disp_msg,
                       sensor_msgs::ImagePtr&            depth_msg);



  /**
   * @brief       Calculates the fraction a given timestamp is located between
   * the closest previous and next timestamp
   *
   * @param[in]  prev_timestamp   Closest previous timestamp
   * @param[in]  query_timestamp  The timestamp for which to compute alpha
   * @param[in]  next_timestamp   Closest next timestampe
   *
   * @return      Fraction
   */
  float compute_alpha(const int64_t prev_timestamp,
                      const int64_t query_timestamp,
                      const int64_t next_timestamp);



  /**
   * @brief      Linear 3D interpolation
   *
   * @param[in]  prev_pos  The previous pose
   * @param[in]  prev_pos  The next pose
   * @param[in]  alpha     Fraction
   *
   * @return     Interpolated translation
   */
  Eigen::Vector3f interpolate_position(const Eigen::Vector3f& prev_pos,
                                       const Eigen::Vector3f& next_pos,
                                       const float            alpha);



  /**
   * @brief      Slerp interpolation for quaterniond
   *
   * @param[in]  prev_orientation  The previous orientation
   * @param[in]  next_orientation  The next orientation
   * @param[in]  alpha             Fraction
   *
   * @return     Interpolated orientation
   */
  Eigen::Quaternionf interpolate_orientation(
      const Eigen::Quaternionf& prev_orientation,
      const Eigen::Quaternionf& next_orientation,
      const float               alpha);

  /**
   * @brief      Interpolation for transformations
   *
   * @param[in]  prev_pose        The previous pose
   * @param[in]  next_pose        The next pose
   * @param[in]  query_timestamp  The timestamp the pose should interpolated at
   *
   * @return     Interpolated pose
   */
  Eigen::Matrix4f interpolate_pose(
      const geometry_msgs::TransformStamped& prev_pose,
      const geometry_msgs::TransformStamped& next_pose,
      const int64_t                          query_timestamp);

} // namespace se

#endif

