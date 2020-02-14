//
// Created by anna on 16/05/19.
//

#ifndef __UTILITIES_HPP
#define __UTILITIES_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>



namespace se {
  /**
   * @brief The possible return values of se::get_surrounding_poses().
   */
  enum InterpResult {
    /**
     * @brief The interpolation was successful.
     * This is also the case when a pose with the exact same timestamp as the
     * query is found, in which case no interpolation is needed.
     */
    ok            = 0,
    /**
     * @brief The query timestamp is smaller than all the pose timestamps.
     * No interpolation could be performed.
     */
    query_smaller = 1,
    /**
     * @brief The query timestamp is great than all the pose timestamps.
     * No interpolation could be performed.
     */
    query_greater = 2,
  };



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
   * @param image_size
   */
  void createImageMsg(const sensor_msgs::ImageConstPtr& old_image_msg,
                      sensor_msgs::ImagePtr&            new_image_msg,
                      Eigen::Vector2i&                  image_size);



  /**
   * @brief swaps the rotation axis to convert the world frame to octree frame
   * @param[in] camera pose in world frame
   * @return rotated pose
   */
  Eigen::Matrix4f swapAxes(const Eigen::Matrix4f& input);



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



  void print_timings(
      const std::vector<std::chrono::time_point<std::chrono::steady_clock> >& timings,
      const std::vector<std::string>& labels);



  /**
   * @brief Find the two closest poses whose timestamps are before and after the
   * query timestamp.
   *
   * @param[in]  buffer          The circular buffer containing all the poses.
   * @param[in]  query_timestamp The timestamp to look for.
   * @param[out] prev_pose       The pose whose timestamp is exactly before the
   *                             query timestamp.
   * @param[out] next_pose       The pose whose timestamp is exactly after the
   *                             query timestamp.
   *
   * @return An se::InterpResult. See its documentation for details.
   */
  InterpResult get_surrounding_poses(
      const boost::circular_buffer<geometry_msgs::TransformStamped>& buffer,
      const uint64_t                                                 query_timestamp,
      geometry_msgs::TransformStamped&                               prev_pose,
      geometry_msgs::TransformStamped&                               next_pose);

} // namespace se

#endif

