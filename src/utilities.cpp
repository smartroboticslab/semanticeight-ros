//
// Created by anna on 16/05/19.
//

#include "supereight_ros/utilities.hpp"



namespace se {
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

} // namespace se

