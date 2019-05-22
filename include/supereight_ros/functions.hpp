//
// Created by anna on 16/05/19.
//

#ifndef EXPLORATION_WS_FUNCTIONS_HPP
#define EXPLORATION_WS_FUNCTIONS_HPP

#include <supereight_ros/supereight_ros.hpp>

namespace se {
/**
 * @brief     converts disparity image from gazebo firefly with vi_sensor to depth image accepted
 * by supereight
 * @param[in] disp_msg disparity image to be converted
 * @param[in] depth_msg destination of depth image
 */

void ConvertDisp2Depth(const sensor_msgs::ImageConstPtr &disp_msg,
                                          sensor_msgs::ImagePtr &depth_msg, const float &constant) {
  // for each depth Z = focal length*baseline/disparity

  const float *disp_row = reinterpret_cast<const float *>(&disp_msg->data[0]);
  int row_step = disp_msg->step / sizeof(float);
  float *depth_data = reinterpret_cast<float *>(&depth_msg->data[0]);
  for (int v = 0; v < (int) disp_msg->height; ++v) {
    for (int u = 0; u < (int) disp_msg->width; ++u) {
      float disp = disp_row[u];
      if (std::isfinite(disp)) {
        *depth_data = constant / disp; //[px][ m]/[px]
      }
      ++depth_data;
    }
    disp_row += row_step;
  }
//  std::cout <<"depth "<< depth_data[320+640*240] <<std::endl;
}
/**
 * @brief create new image message with all attributes
 * @param old_image_msg
 * @param new_image_msg
 */
void CreateImageMsg(const sensor_msgs::ImageConstPtr &old_image_msg, sensor_msgs::ImagePtr &new_image_msg){
  new_image_msg->header = old_image_msg->header;
  new_image_msg->width = old_image_msg->width;
  new_image_msg->height = old_image_msg->height;
  new_image_msg->step = sizeof(float) * new_image_msg->width;
  new_image_msg->encoding = old_image_msg->encoding;

}
/**
 * @brief create new image message with all attributes
 * @param old_image_msg
 * @param new_image_msg
 * @param image_size
 */
void CreateImageMsg(const supereight_ros::ImagePose::ConstPtr &old_image_msg, sensor_msgs::ImagePtr &new_image_msg, Eigen::Vector2i image_size){
  new_image_msg->header = old_image_msg->image.header;
  new_image_msg->width = image_size.x();
  new_image_msg->height = image_size.y();
  new_image_msg->step = sizeof(float) * image_size.x(); //sizeof(float)=4
  new_image_msg->data = std::vector<uint8_t>(image_size.x() * image_size.y() * sizeof(float));
}

/**
 * @brief converts the world pose to octree frame pose
 * @param input
 * @return
 */
Eigen::Matrix4f SwapAxes_octree_world(const Eigen::Matrix4f &input){
  Eigen::Matrix4f output = input;
  output.block<3,1>(0,0) = -input.block<3,1>(0,1);
  output.block<3,1>(0,1) = -input.block<3,1>(0,2);
  output.block<3,1>(0,2) = input.block<3,1>(0,0);
  return output;
}

bool lookupTransform(const std::string& from_frame,
                     const std::string& to_frame, const ros::Time& timestamp,
                     Transformation* transform){
    if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(from_frame, to_frame, timestamp, transform);
  }
}
bool lookupTransformTf(const std::string& from_frame,
                       const std::string& to_frame,
                       const ros::Time& timestamp, Transformation* transform){
  tf::StampedTransform tf_transform;

  ros::Time time_to_lookup = timestamp;
  try {
    tf_listener_.waitForTransform(to_frame, from_frame, time_to_lookup, tf_update_latency);
    tf_listener_.lookupTransform(to_frame, from_frame, time_to_lookup,
                                 tf_transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;


}
bool lookupTransformQueue(const std::string& from_frame,
                          const std::string& to_frame,
                          const ros::Time& timestamp,
                          Transformation* transform){

  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it->header.stamp > timestamp) {
      if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  if (match_found) {
    Transformation T_G_D;
    tf::transformMsgToKindr(it->transform, &T_G_D);

    // If we have a static transform, apply it too.
    // Transform should actually be T_G_C. So need to take it through the full
    // chain.
    *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

    // And also clear the queue up to this point. This leaves the current
    // message in place.
    transform_queue_.erase(transform_queue_.begin(), it);
  } else {
    ROS_WARN_STREAM_THROTTLE(
        30, "No match found for transform timestamp: " << timestamp);
    if (!transform_queue_.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          30,
          "Queue front: " << transform_queue_.front().header.stamp
                          << " back: " << transform_queue_.back().header.stamp);
    }
  }
  return match_found;
}
}
}
#endif //EXPLORATION_WS_FUNCTIONS_HPP
