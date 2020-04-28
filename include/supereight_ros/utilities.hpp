/*
 * SPDX-FileCopyrightText: 2019 Anna Dai
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
  /*!
   * \brief The possible return values of se::get_surrounding_poses().
   */
  enum InterpResult {
    /*!
     * The interpolation was successful.  This is also the case when a pose with
     * the exact same timestamp as the query is found, in which case no
     * interpolation is needed.
     */
    ok            = 0,
    /*!
     * The query timestamp is smaller than all the pose timestamps.  No
     * interpolation could be performed.
     */
    query_smaller = 1,
    /*!
     * The query timestamp is greater than all the pose timestamps.  No
     * interpolation could be performed.
     */
    query_greater = 2,
  };



  /*!
   * \brief Copy the depth image message into the supereight buffer.
   *
   * If needed, the input depth image is converted into uint16_t containing
   * millimiters.
   *
   * \param[in]  input_depth  The depth message to copy.
   * \param[out] output_depth The destination buffer.
   */
  void to_supereight_depth(const sensor_msgs::Image& input_depth,
                           uint16_t*                 output_depth);



  /*!
   * \brief Create an RGBA image message from a buffer.
   *
   * \param[in] image_data    The RGBA image data to copy into the new image.
   * \param[in] image_res     The resolution of the new image.
   * \param[in] header_source The image to copy the header from.
   */
  sensor_msgs::Image msg_from_RGBA_image(
      const uint32_t*                   image_data,
      const Eigen::Vector2i&            image_res,
      const sensor_msgs::ImageConstPtr& header_source);



  /*!
   * \brief Linear pose interpolation.
   *
   * \param[in] prev_pose       The previous pose.
   * \param[in] next_pose       The next pose.
   * \param[in] query_timestamp The timestamp the pose should interpolated at in
   *                            seconds.
   *
   * \return The interpolated pose.
   */
  Eigen::Matrix4f interpolate_pose(
      const geometry_msgs::TransformStamped& prev_pose,
      const geometry_msgs::TransformStamped& next_pose,
      const double                           query_timestamp);



  void print_timings(
      const std::vector<std::chrono::time_point<std::chrono::steady_clock> >& timings,
      const std::vector<std::string>& labels);



  /*!
   * \brief Find the two closest poses whose timestamps are before and after the
   * query timestamp.
   *
   * \param[in]  buffer          The circular buffer containing all the poses.
   * \param[in]  query_timestamp The timestamp to look for in seconds.
   * \param[out] prev_pose       The pose whose timestamp is exactly before the
   *                             query timestamp.
   * \param[out] next_pose       The pose whose timestamp is exactly after the
   *                             query timestamp.
   *
   * \return An se::InterpResult.
   */
  InterpResult get_surrounding_poses(
      const boost::circular_buffer<geometry_msgs::TransformStamped>& buffer,
      const double                                                   query_timestamp,
      geometry_msgs::TransformStamped&                               prev_pose,
      geometry_msgs::TransformStamped&                               next_pose);



  /*!
   * \brief Find the image in the buffer that is closest to the query_timestamp.
   *
   * The difference between the query_timestamp and the image timestamp will be
   * at most threshold seconds.
   *
   * \param[in]  buffer          The circular buffer containing the images.
   * \param[in]  query_timestamp The timestamp in seconds to try and match.
   * \param[in]  threshold       The maximum time difference in seconds for an
   *                             image to be considered a match.
   * \param[out] closest_image   The matched image.
   *
   * \return true if a match was found, false otherwise.
   */
  bool get_closest_image(
      const boost::circular_buffer<sensor_msgs::ImageConstPtr>& buffer,
      const double                                              query_timestamp,
      const double                                              threshold,
      sensor_msgs::ImageConstPtr&                               closest_image);

} // namespace se

#endif

