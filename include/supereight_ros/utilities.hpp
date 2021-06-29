// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#ifndef __UTILITIES_HPP
#define __UTILITIES_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>

#include "supereight_ros/eigen_ros_conversions.hpp"
#include "se/segmentation_result.hpp"



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



  typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Path;



  static float to_log_odds(const float probability) {
    return std::log2f(probability / (1.0f - probability));
  }



  static float to_probability(const float log_odds) {
    return std::pow(2.0f, log_odds) / (1.0f + std::pow(2.0f, log_odds));
  }



  /*!
   * \brief Copy the depth image message into the supereight buffer.
   * Valid input depth image pixel types are `mono16`, `16UC1` and `32FC1`.
   *
   * \param[in]  input_depth  The depth message to copy.
   * \param[in]  far_plane    The far plane of the depth sensor.
   * \param[out] output_depth The destination buffer.
   */
  void to_supereight_depth(const sensor_msgs::ImageConstPtr& input_depth,
                           const float                       far_plane,
                           float*                            output_depth);



   /*!
   * \brief Copy the color image message into the supereight buffer.
   * Valid input color image pixel types are `rgb8`, `8UC3`, `rgba8` and `8UC4`.
   *
   * \param[in]  input_color The color message to copy.
   * \param[out] output_rgba The destination buffer.
   */
  void to_supereight_RGB(const sensor_msgs::ImageConstPtr& input_color,
                         uint32_t*                         output_rgba);



  SegmentationResult to_supereight_segmentation(const sensor_msgs::ImageConstPtr& input_class,
                                                const sensor_msgs::ImageConstPtr& input_instance);



  /*!
   * \brief Create an image message from an RGBA image.
   *
   * \param[in] image_data The RGBA image data to copy into the message.
   * \param[in] image_res  The resolution of RGBA image.
   * \param[in] header     The header to use for the resulting message.
   */
  sensor_msgs::Image RGBA_to_msg(const uint32_t*         image_data,
                                 const Eigen::Vector2i&  image_res,
                                 const std_msgs::Header& header);



  nav_msgs::Path path_to_msg(const se::Path&         path_WC,
                             const Eigen::Matrix4f&  T_CB,
                             const std_msgs::Header& header);



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



  void print_timings(const std::vector<double>& timings, const std::vector<std::string>& labels);



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
   * \brief Find the element in the buffer that is closest to the query_timestamp.
   *
   * The difference between the query_timestamp and the image timestamp will be
   * at most threshold seconds.
   *
   * \param[in]  buffer          The circular buffer.
   * \param[in]  query_timestamp The timestamp in seconds to try and match.
   * \param[in]  threshold       The maximum time difference in seconds for an
   *                             element to be considered a match.
   * \param[in]  get_timestamp   A function used to get each element's timestamp.
   * \param[out] closest_element The matched element.
   *
   * \return true if a match was found, false otherwise.
   */
  template<typename T, typename GetTimestampF>
  bool get_closest_element(const boost::circular_buffer<T>& buffer,
                           const double                     query_timestamp,
                           const double                     threshold,
                           GetTimestampF                    get_timestamp,
                           T&                               closest_element);



  /*!
   * \brief Convert a transform message to an Eigen matrix.
   *
   * \param[in] tf_msg The transform message to convert.
   *
   * \return An Eigen matrix containing a homogeneous transform.
   */
  Eigen::Matrix4f transform_msg_to_eigen(const geometry_msgs::TransformStamped& tf_msg);



  /*!
   * \brief Convert a pose message to an Eigen matrix.
   *
   * \param[in] pose_msg The pose message to convert.
   *
   * \return An Eigen matrix containing the pose as homogeneous transform.
   */
  Eigen::Matrix4f pose_msg_to_eigen(const geometry_msgs::PoseStamped& pose_msg);

} // namespace se

#include "utilities_impl.hpp"

#endif

