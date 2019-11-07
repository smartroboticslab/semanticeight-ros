//
// Created by anna on 16/05/19.
//

#include "supereight_ros/utilities.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>



// Default supereight configuration.
constexpr int default_iteration_count = 3;
constexpr int default_iterations[default_iteration_count] = {10, 5, 4};
constexpr int default_compute_size_ratio = 2;
constexpr int default_tracking_rate = 1;
constexpr int default_integration_rate = 3;
constexpr int default_rendering_rate = 4;
const Eigen::Vector3i default_volume_resolution(256, 256, 256);
const Eigen::Vector3f default_volume_size(6.f, 6.f, 6.f);
const Eigen::Vector3f default_initial_pos_factor(0.5f, 0.5f, 0.0f);
const std::string default_dump_volume_file = "";
const std::string default_input_file = "";
const std::string default_log_file = "";
const std::string default_groundtruth_file = "";
const Eigen::Matrix4f default_gt_transform = Eigen::Matrix4f::Identity();
constexpr float default_mu = 0.1f;
constexpr int default_fps = 0;
constexpr bool default_blocking_read = false;
constexpr float default_icp_threshold = 1e-6;
constexpr bool default_no_gui = false;
constexpr bool default_render_volume_fullsize = false;
constexpr bool default_bilateral_filter = false;
constexpr int default_coloured_voxels = false;
constexpr int default_multi_resolution = false;
constexpr bool default_bayesian = false;



namespace se {
  Configuration read_supereight_config(const ros::NodeHandle& nh) {
    Configuration config;

    nh.param<int>("compute_size_ratio",
        config.compute_size_ratio,
        default_compute_size_ratio);

    nh.param<int>("tracking_rate",
        config.tracking_rate,
        default_tracking_rate);

    nh.param<int>("integration_rate",
        config.integration_rate,
        default_integration_rate);

    nh.param<int>("rendering_rate",
        config.rendering_rate,
        default_rendering_rate);

    std::vector<int> volume_resolution_vector;
    if (nh.getParam("volume_resolution", volume_resolution_vector)) {
      for (size_t i = 0; i < volume_resolution_vector.size(); ++i) {
        config.volume_resolution[i] = volume_resolution_vector[i];
      }
    } else {
      config.volume_resolution = default_volume_resolution;
    }

    std::vector<float> volume_size_vector;
    if (nh.getParam("volume_size", volume_size_vector)) {
      for (size_t i = 0; i < volume_size_vector.size(); ++i) {
        config.volume_size[i] = volume_size_vector[i];
      }
    } else {
      config.volume_size = default_volume_size;
    }

    std::vector<float> initial_pos_factor_vector;
    if (nh.getParam("initial_pos_factor", initial_pos_factor_vector)) {
      for (size_t i = 0; i < initial_pos_factor_vector.size(); ++i) {
        config.initial_pos_factor[i] = initial_pos_factor_vector[i];
      }
    } else {
      config.initial_pos_factor = default_initial_pos_factor;
    }

    std::vector<int> pyramid;
    if (!nh.getParam("pyramid", pyramid)) {
      config.pyramid.clear();
      for (int i = 0; i < default_iteration_count; ++i) {
        config.pyramid.push_back(default_iterations[i]);
      }
    }

    nh.param<std::string>("dump_volume_file",
        config.dump_volume_file,
        default_dump_volume_file);

    nh.param<std::string>("input_file",
        config.input_file,
        default_input_file);

    nh.param<std::string>("log_file",
        config.log_file,
        default_log_file);

    nh.param<std::string>("groundtruth_file",
        config.groundtruth_file,
        default_groundtruth_file);

    std::vector<float> gt_transform_vector;
    nh.getParam("gt_transform", gt_transform_vector);
    if (nh.getParam("gt_transform", gt_transform_vector)) {
      for (size_t i = 0; i < std::sqrt(gt_transform_vector.size()); ++i) {
        for (size_t j = 0; j < std::sqrt(gt_transform_vector.size()); ++j) {
          config.gt_transform(i, j) = gt_transform_vector[i * 4 + j];
        }
      }
    } else {
      config.gt_transform = default_gt_transform;
    }

    std::vector<float> camera_vector;
    if (!nh.getParam("camera", camera_vector)) {
      ros::shutdown();
    }
    for (size_t i = 0; i < camera_vector.size(); i++) {
      config.camera[i] = camera_vector[i];
    }
    config.camera_overrided = true;

    nh.param<float>("mu",
        config.mu,
        default_mu);

    nh.param<int>("fps",
        config.fps,
        default_fps);

    nh.param<bool>("blocking_read",
        config.blocking_read,
        default_blocking_read);

    nh.param<float>("icp_threshold",
        config.icp_threshold,
        default_icp_threshold);

    nh.param<bool>("no_gui",
        config.no_gui,
        default_no_gui);

    nh.param<bool>("render_volume_fullsize",
        config.render_volume_fullsize,
        default_render_volume_fullsize);

    nh.param<bool>("bilateral_filter",
        config.bilateral_filter,
        default_bilateral_filter);

    nh.param<bool>("coloured_voxels",
        config.coloured_voxels,
        default_coloured_voxels);

    nh.param<bool>("multi_resolution",
        config.multi_resolution,
        default_multi_resolution);

    nh.param<bool>("bayesian",
        config.bayesian,
        default_bayesian);

    return config;
  }



  void print_supereight_config(const Configuration& config) {
    ROS_INFO("Supereight parameters:");
    ROS_INFO("  compute_size_ratio:     %d",
        config.compute_size_ratio);
    ROS_INFO("  tracking_rate:          %d",
        config.tracking_rate);
    ROS_INFO("  integration_rate:       %d",
        config.integration_rate);
    ROS_INFO("  rendering_rate:         %d",
        config.rendering_rate);
    ROS_INFO("  volume_resolution:      %d %d %d",
        config.volume_resolution.x(),
        config.volume_resolution.y(),
        config.volume_resolution.z());
    ROS_INFO("  volume_size:            %f %f %f",
        config.volume_size.x(),
        config.volume_size.y(),
        config.volume_size.z());
    ROS_INFO("  initial_pos_factor:     %f %f %f",
        config.initial_pos_factor.x(),
        config.initial_pos_factor.y(),
        config.initial_pos_factor.z());
    ROS_INFO("  pyramid:                %d %d %d",
        config.pyramid[0],
        config.pyramid[1],
        config.pyramid[2]);
    ROS_INFO("  dump_volume_file:       \"%s\"",
        config.dump_volume_file.c_str());
    ROS_INFO("  input_file:             \"%s\"",
        config.input_file.c_str());
    ROS_INFO("  log_file:               \"%s\"",
        config.log_file.c_str());
    ROS_INFO("  groundtruth_file:       \"%s\"",
        config.groundtruth_file.c_str());
    ROS_INFO("  gt_transform:");
    for (size_t i = 0; i < 4; ++i) {
      ROS_INFO("                          %f %f %f %f",
          config.gt_transform(4 * i + 0),
          config.gt_transform(4 * i + 1),
          config.gt_transform(4 * i + 2),
          config.gt_transform(4 * i + 3));
    }
    ROS_INFO("  camera:                 %f %f %f %f",
        config.camera.x(),
        config.camera.y(),
        config.camera.z(),
        config.camera.w());
    ROS_INFO("  camera_overrided:       %d",
        config.camera_overrided);
    ROS_INFO("  mu:                     %f",
        config.mu);
    ROS_INFO("  fps:                    %d",
        config.fps);
    ROS_INFO("  blocking_read:          %d",
        config.blocking_read);
    ROS_INFO("  icp_threshold:          %f",
        config.icp_threshold);
    ROS_INFO("  no_gui:                 %d",
        config.no_gui);
    ROS_INFO("  render_volume_fullsize: %d",
        config.render_volume_fullsize);
    ROS_INFO("  bilateral_filter:       %d",
        config.bilateral_filter);
    ROS_INFO("  coloured_voxels:        %d",
        config.coloured_voxels);
    ROS_INFO("  multi_resolution:       %d",
        config.multi_resolution);
    ROS_INFO("  bayesian:               %d",
        config.bayesian);
  }



  void to_supereight_depth(const sensor_msgs::Image& input_depth,
                           uint16_t*                 output_depth) {

    if (input_depth.encoding == "16UC1") {
      // Just copy the image data since this is already the correct format.
      const size_t image_size_bytes = input_depth.height * input_depth.step;
      std::memcpy(output_depth, input_depth.data.data(), image_size_bytes);

    } else if (input_depth.encoding == "32FC1") {
      // The depth is in float meters, convert to uint16_t millimeters.
      const size_t image_size_pixels = input_depth.width * input_depth.height;
      // Cast the image data as a float pointer so that operator[] can be used
      // to get the value of each pixel.
      const float* input_ptr
          = reinterpret_cast<const float*>(input_depth.data.data());
#pragma omp parallel for
      for (size_t i = 0; i < image_size_pixels; ++i) {
        const float depth_mm = 1000.f * input_ptr[i];
        // Test whether the depth value is NaN or if it would cause an overflow
        // in a uint16_t. In that case store an invalid depth value.
        if (std::isnan(depth_mm) || (depth_mm > (1 << 16) - 1)) {
          output_depth[i] = 0;
        } else {
          output_depth[i] = static_cast<uint16_t>(depth_mm);
        }
      }

    } else {
      ROS_FATAL_STREAM("Invalid input depth format: " << input_depth.encoding);
      abort();
    }
  }



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



  float compute_alpha(const int64_t prev_timestamp,
                      const int64_t query_timestamp,
                      const int64_t next_timestamp) {

    return static_cast<float>(query_timestamp - prev_timestamp)
         / static_cast<float>(next_timestamp - prev_timestamp);
  }



  Eigen::Vector3f interpolate_position(const Eigen::Vector3f& prev_pos,
                                       const Eigen::Vector3f& next_pos,
                                       const float            alpha) {

    return prev_pos + alpha * (next_pos - prev_pos);
  }



  Eigen::Quaternionf interpolate_orientation(
      const Eigen::Quaternionf& prev_orientation,
      const Eigen::Quaternionf& next_orientation,
      const float               alpha) {

    return prev_orientation.slerp(alpha, next_orientation);
  }



  Eigen::Matrix4f interpolate_pose(
      const geometry_msgs::TransformStamped& prev_pose,
      const geometry_msgs::TransformStamped& next_pose,
      const int64_t                          query_timestamp) {

    const float alpha = compute_alpha(
        ros::Time(prev_pose.header.stamp).toNSec(),
        query_timestamp,
        ros::Time(next_pose.header.stamp).toNSec());

    const Eigen::Vector3f prev_translation(
        prev_pose.transform.translation.x,
        prev_pose.transform.translation.y,
        prev_pose.transform.translation.z);
    const Eigen::Vector3f next_translation(
        next_pose.transform.translation.x,
        next_pose.transform.translation.y,
        next_pose.transform.translation.z);

    const Eigen::Quaternionf prev_rotation(
        prev_pose.transform.rotation.w,
        prev_pose.transform.rotation.x,
        prev_pose.transform.rotation.y,
        prev_pose.transform.rotation.z);
    const Eigen::Quaternionf next_rotation(
        next_pose.transform.rotation.w,
        next_pose.transform.rotation.x,
        next_pose.transform.rotation.y,
        next_pose.transform.rotation.z);

    const Eigen::Vector3f inter_translation
        = interpolate_position(prev_translation, next_translation, alpha);
    const Eigen::Quaternionf inter_rotation
        = interpolate_orientation(prev_rotation, next_rotation, alpha);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = inter_translation;
    pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

    return pose;
  }

} // namespace se

