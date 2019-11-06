//
// Created by anna on 16/05/19.
//

#ifndef __UTILITIES_HPP
#define __UTILITIES_HPP

#include "supereight_ros/supereight_ros.hpp"

namespace se {

/**
 * @brief create new image message with all attributes
 * @param old_image_msg
 * @param new_image_msg
 */
void createImageMsg(const sensor_msgs::ImageConstPtr &old_image_msg,
                    sensor_msgs::ImagePtr &new_image_msg);
/**
 * @brief create new image message with all attributes
 * @param old_image_msg
 * @param new_image_msg
 * @param image_size
 */
void createImageMsg(const supereight_ros::ImagePose::ConstPtr &old_image_msg,
                    sensor_msgs::ImagePtr &new_image_msg,
                    Eigen::Vector2i image_size);


/**
 * @brief swaps the rotation axis to convert the world frame to octree frame
 * @param[in] camera pose in world frame
 * @return rotated pose
 */
Eigen::Matrix4f swapAxes(const Eigen::Matrix4f &input);

void createTestImage(const sensor_msgs::ImageConstPtr &disp_msg,
                       sensor_msgs::ImagePtr &depth_msg);
}
#endif //EXPLORATION_WS_FUNCTIONS_HPP
