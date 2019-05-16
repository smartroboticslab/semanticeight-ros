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
}
#endif //EXPLORATION_WS_FUNCTIONS_HPP
