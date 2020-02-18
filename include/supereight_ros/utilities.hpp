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
   * @brief Copy the depth image message into the supereight buffer.
   * If needed, the input depth image is converted into uint16_t containing
   * millimiters.
   *
   * @param[in]  input_depth  The depth message to copy.
   * @param[out] output_depth The destination buffer.
   */
  void to_supereight_depth(const sensor_msgs::Image& input_depth,
                           uint16_t*                 output_depth);



  /**
   * @brief Create an RGB image message from a buffer.
   *
   * @param[in] image_data    The data to copy into the new image.
   * @param[in] image_size    The dimensions of the new image.
   * @param[in] header_source The image to copy the header from.
   */
  sensor_msgs::Image msg_from_RGB_image(
      const uint32_t*                   image_data,
      const Eigen::Vector2i&            image_size,
      const sensor_msgs::ImageConstPtr& header_source);



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



  /**
   * @brief Find the image in the buffer that is closest to the query_timestamp.
   * The difference between the query_timestamp and the image timestamp will be
   * at most threshold seconds.
   *
   * @param[in]  buffer          The circular buffer containing the images.
   * @param[in]  query_timestamp The timestamp in seconds to try and match.
   * @param[in]  threshold       The maximum time difference in seconds for an
   *                             image to be considered a match.
   * @param[out] closest_image   The matched image.
   *
   * @return true if a match was found, false otherwise.
   */
  bool get_closest_image(
      const boost::circular_buffer<sensor_msgs::ImageConstPtr>& buffer,
      const double                                              query_timestamp,
      const double                                              threshold,
      sensor_msgs::ImageConstPtr&                               closest_image);

} // namespace se

#endif

