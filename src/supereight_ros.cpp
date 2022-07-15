// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include "supereight_ros/supereight_ros.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <lodepng.h>
#include <map>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "se/io/meshing_io.hpp"
#include "se/system_info.hpp"
#include "se/voxel_implementations.hpp"
#include "supereight_ros/filesystem.hpp"
#include "supereight_ros/utilities.hpp"



auto get_image_timestamp = [](sensor_msgs::ImageConstPtr img) { return img->header.stamp; };

namespace se {

SupereightNode::SupereightNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
        nh_(nh),
        nh_private_(nh_private),
        sensor_({1, 1, false, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}),
        frame_(-1),
        num_planning_iterations_(0),
        num_failed_planning_iterations_(0),
        max_failed_planning_iterations_(0),
        input_segmentation_(0, 0),
#ifdef SE_WITH_MASKRCNN
        network_(network_config_),
#endif // SE_WITH_MASKRCNN
        tf_listener_(tf_buffer_),
        keep_running_(true),
        world_frame_id_("world"),
        map_frame_id_("map"),
        body_frame_id_("body"),
        camera_frame_id_("camera")
{
    // Set the ROS_LOG_DIR environment variable to allow expanding it in YAML files.
    setenv("ROS_LOG_DIR", ros_log_dir().c_str(), 0);
    readConfig(nh_private);
    if (node_config_.dataset == Dataset::Gazebo) {
        body_frame_id_ = "firefly/base_link";
        camera_frame_id_ = "firefly/vi_sensor/camera_depth_optical_center_link";
    }
    else if (node_config_.dataset == Dataset::Real) {
        world_frame_id_ = "vicon/world";
        body_frame_id_ = "vicon/s550_jetson/s550_jetson";
    }

    T_MW_ = Eigen::Matrix4f::Identity();
    T_MW_.topRightCorner<3, 1>() =
        supereight_config_.t_MW_factor.cwiseProduct(supereight_config_.map_dim);
    T_WM_ = se::math::to_inverse_transformation(T_MW_);
    T_CB_ = supereight_config_.T_BC.inverse();
    init_t_WB_ = Eigen::Vector3f::Constant(NAN);
    image_res_ = node_config_.input_res / supereight_config_.sensor_downsampling_factor;

    // Allocate input image buffers.
    const size_t input_num_pixels = node_config_.input_res.prod();
    input_depth_ = std::unique_ptr<float[]>(new float[input_num_pixels]);
    if (node_config_.enable_rgb) {
        input_rgba_ = std::unique_ptr<uint32_t[]>(new uint32_t[input_num_pixels]);
    }

    // Allocate rendered image buffers.
    if (node_config_.enable_rendering) {
        const size_t render_num_pixels = image_res_.prod();
        depth_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        if (node_config_.enable_rgb) {
            rgba_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        }
        if (node_config_.enable_tracking) {
            track_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        }
        volume_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        volume_render_color_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        volume_render_scale_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        volume_render_min_scale_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        if (node_config_.enable_objects) {
            class_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
            instance_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
        }
        raycast_render_ = std::unique_ptr<uint32_t[]>(new uint32_t[render_num_pixels]);
    }

    // Initialize the sensor.
    const Eigen::Vector2i downsampled_res =
        node_config_.input_res / supereight_config_.sensor_downsampling_factor;
    const Eigen::VectorXf elevation_angles = (Eigen::VectorXf(64) << 17.74400,
                                              17.12000,
                                              16.53600,
                                              15.98200,
                                              15.53000,
                                              14.93600,
                                              14.37300,
                                              13.82300,
                                              13.37300,
                                              12.78600,
                                              12.23000,
                                              11.68700,
                                              11.24100,
                                              10.67000,
                                              10.13200,
                                              9.57400,
                                              9.13800,
                                              8.57700,
                                              8.02300,
                                              7.47900,
                                              7.04600,
                                              6.48100,
                                              5.94400,
                                              5.39500,
                                              4.96300,
                                              4.40100,
                                              3.85900,
                                              3.31900,
                                              2.87100,
                                              2.32400,
                                              1.78300,
                                              1.23800,
                                              0.78600,
                                              0.24500,
                                              -0.29900,
                                              -0.84900,
                                              -1.28800,
                                              -1.84100,
                                              -2.27500,
                                              -2.92600,
                                              -3.37800,
                                              -3.91000,
                                              -4.45700,
                                              -5.00400,
                                              -5.46000,
                                              -6.00200,
                                              -6.53700,
                                              -7.09600,
                                              -7.55200,
                                              -8.09000,
                                              -8.62900,
                                              -9.19600,
                                              -9.65700,
                                              -10.18300,
                                              -10.73200,
                                              -11.28900,
                                              -11.77000,
                                              -12.29700,
                                              -12.85400,
                                              -13.41500,
                                              -13.91600,
                                              -14.44200,
                                              -14.99700,
                                              -15.59500)
                                                 .finished();
    const Eigen::VectorXf azimuth_angles = (Eigen::VectorXf(64) << 0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)
                                               .finished();
    sensor_ = SensorImpl(
        {downsampled_res.x(),
         downsampled_res.y(),
         supereight_config_.left_hand_frame,
         supereight_config_.near_plane,
         supereight_config_.far_plane,
         supereight_config_.sensor_intrinsics[0] / supereight_config_.sensor_downsampling_factor,
         supereight_config_.sensor_intrinsics[1] / supereight_config_.sensor_downsampling_factor,
         supereight_config_.sensor_intrinsics[2] / supereight_config_.sensor_downsampling_factor,
         supereight_config_.sensor_intrinsics[3] / supereight_config_.sensor_downsampling_factor,
         supereight_config_.sensor_distortion[0],
         supereight_config_.sensor_distortion[1],
         supereight_config_.sensor_distortion[2],
         supereight_config_.sensor_distortion[3],
         azimuth_angles,
         elevation_angles});

    // Initialize the supereight pipeline.
    pipeline_ = std::shared_ptr<DenseSLAMSystem>(
        new DenseSLAMSystem(image_res_,
                            Eigen::Vector3i::Constant(supereight_config_.map_size.x()),
                            Eigen::Vector3f::Constant(supereight_config_.map_dim.x()),
                            T_MW_,
                            supereight_config_.pyramid,
                            supereight_config_));
    planner_ = std::unique_ptr<se::ExplorationPlanner>(
        new se::ExplorationPlanner(*pipeline_, sensor_, supereight_config_));

    // Allocate message circular buffers.
    if (node_config_.enable_tracking) {
        pose_buffer_.set_capacity(0);
    }
    else {
        pose_buffer_.set_capacity(node_config_.pose_buffer_size);
    }
    depth_buffer_.set_capacity(node_config_.depth_buffer_size);
    if (node_config_.enable_rgb) {
        rgb_buffer_.set_capacity(node_config_.rgb_buffer_size);
        if (!node_config_.run_segmentation && node_config_.enable_objects) {
            class_buffer_.set_capacity(node_config_.rgb_buffer_size);
            instance_buffer_.set_capacity(node_config_.rgb_buffer_size);
        }
    }
    else {
        rgb_buffer_.set_capacity(0);
        class_buffer_.set_capacity(0);
        instance_buffer_.set_capacity(0);
    }

    // Use the correct classes.
    switch (node_config_.dataset) {
    case Dataset::Replica:
        se::semantic_classes = se::SemanticClasses::replica_classes();
        break;
    case Dataset::Habitat:
    case Dataset::Matterport3D:
        se::semantic_classes = se::SemanticClasses::matterport3d_classes();
        break;
    case Dataset::Real:
    case Dataset::Gazebo:
    default:
        se::semantic_classes = se::SemanticClasses::coco_classes();
    }
    // Override classes when running Mask R-CNN.
    if (node_config_.run_segmentation) {
        se::semantic_classes = se::SemanticClasses::coco_classes();
    }
    if (node_config_.dataset == Dataset::Real) {
        se::semantic_classes.setEnabled("backpack");
        se::semantic_classes.setEnabled("book");
        se::semantic_classes.setEnabled("cup");
        se::semantic_classes.setEnabled("keyboard");
    }
    else {
        se::semantic_classes.setEnabled("chair");
    }
    se::semantic_classes.setResAll(0.01f);

    if (node_config_.run_segmentation) {
#ifdef SE_WITH_MASKRCNN
        network_config_.model_filename = "/home/srl/Documents/Datasets/MaskRCNN/mrcnn_nchw.uff";
        network_config_.serialized_model_filename = network_config_.model_filename + ".bin";
        network_ = mr::MaskRCNNConfig(network_config_);
        if (!network_.build()) {
            ROS_FATAL("Couldn't initialize the network.");
            ros::shutdown();
        }
#else
        ROS_FATAL("Not compiled with Mask R-CNN support, run_segmentation must be set to false.");
        ros::shutdown();
#endif // SE_WITH_MASKRCNN
    }

    setupRos();

    exploration_start_time_ = std::chrono::steady_clock::now();
    ROS_INFO("Initialization finished");

    // Rotors Control interface
    if (node_config_.dataset == Dataset::Gazebo
        && node_config_.control_interface == ControlInterface::RotorS) {
        for (double i = 0.0; i <= 0.2; i += 0.05) {
            mav_msgs::EigenTrajectoryPoint point;
            nh.param<double>("wp_x", point.position_W.x(), 0.0);
            nh.param<double>("wp_y", point.position_W.y(), 0.0);
            nh.param<double>("wp_z", point.position_W.z(), 1.0);

            // Take the constant tracking error of the controller into account
            point.position_W.z() += 0.2;
            tf::Quaternion q_tf_WB = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
            double yaw = tf::getYaw(q_tf_WB);
            point.setFromYaw(yaw);

            trajectory_msgs::MultiDOFJointTrajectory path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.points.clear();

            trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(point, &point_msg);
            path_msg.points.push_back(point_msg);
            path_pub_.publish(path_msg);
        }
    }
    else if (node_config_.dataset == Dataset::Gazebo
             && node_config_.control_interface == ControlInterface::SRL) {
        // Desired position for the beginning of the experiment
        Eigen::Vector3d desiredPosition;
        nh.param<double>("wp_x", desiredPosition.x(), 0.0);
        nh.param<double>("wp_y", desiredPosition.y(), 0.0);
        nh.param<double>("wp_z", desiredPosition.z(), 1.0);

        // Create a take off trajectory and an initial Waypoint. Use 0 altitude for the the initial Waypoint.
        mav_interface_msgs::FullStateStampedEigen initialStateEigen(
            0, Eigen::Vector3d(desiredPosition(0), desiredPosition(1), 0.0));

        // Create an initial waypoint at the exact same location as the starting state of the take off trajectory. Use large
        // tolerances to pop the initial Waypoint immediately
        mav_interface_msgs::WaypointEigen initialWaypointEigen(
            desiredPosition, initialStateEigen.orientation, 0.5, 0.5);

        // Take off trajectory will last 4.0 seconds. ToDo -> Use the exploration velocity for these.
        const uint64_t takeOffDurationNanoseconds = 4.0 * 1e+9;
        mav_interface_msgs::FullStateStampedEigen finalStateEigen(takeOffDurationNanoseconds,
                                                                  desiredPosition);

        // Convert the above to ROS messages and asseble the trajectory.
        mav_interface_msgs::Waypoint waypointMsg;
        mav_interface_msgs::FullStateStamped initialStateMsg;
        mav_interface_msgs::FullStateStamped finalStateMsg;

        WaypointEigen2Msg(initialWaypointEigen, waypointMsg);
        FullStateStampedEigen2Msg(initialStateEigen, initialStateMsg);
        FullStateStampedEigen2Msg(finalStateEigen, finalStateMsg);

        mav_interface_msgs::FullStateTrajectory trajectoryMsg;
        trajectoryMsg.header.stamp = ros::Time::now();
        trajectoryMsg.initialWaypoint = waypointMsg;
        trajectoryMsg.trajectory.push_back(initialStateMsg);
        trajectoryMsg.trajectory.push_back(finalStateMsg);

        // Publish
        // ros::Duration(1.0).sleep();
        // path_pub_.publish(trajectoryMsg);
        ros::Duration(4.0).sleep();
    }
    else {
        // Placeholder for Real life experiments
    }

    stats_ = Stats(supereight_config_.log_path);

    // Start the matching thread.
    matching_thread_ = std::thread(std::bind(&SupereightNode::matchAndFuse, this));

    // Start the planner thread.
    if (supereight_config_.enable_exploration) {
        planning_thread_ = std::thread(std::bind(&SupereightNode::plan, this));
    }
}



SupereightNode::~SupereightNode()
{
    // Wait for all threads to finish
    keep_running_ = false;
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
    if (matching_thread_.joinable()) {
        matching_thread_.join();
    }
    // Save the map to a file if needed
    saveMap();
    ROS_INFO("supereight_ros node stopped successfully");
}



void SupereightNode::matchAndFuse()
{
    while (keep_running_) {
        // matchAndFuse() should only be run by a single thread at a time. Return
        // if the lock can't be acquired (another thread is already running).
        std::unique_lock<std::mutex> matching_lock(matching_mutex_, std::defer_lock_t());
        if (!matching_lock.try_lock()) {
            continue;
        }

        std::chrono::time_point<std::chrono::steady_clock> start_time;
        std::chrono::time_point<std::chrono::steady_clock> end_time;
        start_time = std::chrono::steady_clock::now();



        // Message association
        // Depth
        sensor_msgs::ImageConstPtr current_depth_msg;
        ros::Time depth_timestamp;
        { // Block to reduce the scope of depth_lock.
            const std::lock_guard<std::mutex> depth_lock(depth_buffer_mutex_);
            if (depth_buffer_.empty()) {
                ROS_DEBUG_THROTTLE_NAMED(5, "matching", "Matching: waiting for depth image");
                continue;
            }
            else {
                current_depth_msg = depth_buffer_.front();
                depth_timestamp = ros::Time(current_depth_msg->header.stamp);
            }
        }

        // RGB
        sensor_msgs::ImageConstPtr current_rgb_msg;
        if (node_config_.enable_rgb) {
            const std::lock_guard<std::mutex> rgb_lock(rgb_buffer_mutex_);
            if (rgb_buffer_.empty()) {
                ROS_DEBUG_THROTTLE_NAMED(5, "matching", "Matching: waiting for colour image");
                continue;
            }
            else {
                const bool found = get_closest_element(rgb_buffer_,
                                                       depth_timestamp,
                                                       node_config_.max_timestamp_diff,
                                                       get_image_timestamp,
                                                       current_rgb_msg);
                if (!found) {
                    ROS_DEBUG_NAMED("matching", "Matching: no matching colour image found");
                    continue;
                }
            }
        }

        bool semantics_found = true;
        // Class
        sensor_msgs::ImageConstPtr current_class_msg;
        if (node_config_.enable_objects) {
            const std::lock_guard<std::mutex> class_lock(class_buffer_mutex_);
            if (class_buffer_.empty()) {
                semantics_found = false;
            }
            else {
                const bool found = get_closest_element(class_buffer_,
                                                       depth_timestamp,
                                                       node_config_.max_timestamp_diff,
                                                       get_image_timestamp,
                                                       current_class_msg);
                if (!found) {
                    semantics_found = false;
                    ROS_WARN_ONCE("No matching semantic class images found");
                }
            }
        }

        // Instance
        sensor_msgs::ImageConstPtr current_instance_msg;
        if (node_config_.enable_objects) {
            const std::lock_guard<std::mutex> instance_lock(instance_buffer_mutex_);
            if (instance_buffer_.empty()) {
                semantics_found = false;
            }
            else {
                const bool found = get_closest_element(instance_buffer_,
                                                       depth_timestamp,
                                                       node_config_.max_timestamp_diff,
                                                       get_image_timestamp,
                                                       current_instance_msg);
                if (!found) {
                    semantics_found = false;
                    ROS_WARN_ONCE("No matching semantic instance images found");
                }
            }
        }

        // Pose
        Eigen::Matrix4f external_T_WC;
        if (!node_config_.enable_tracking) {
            const std::lock_guard<std::mutex> pose_lock(pose_buffer_mutex_);
            if (pose_buffer_.empty()) {
                // Clear the depth and RGB buffers if no poses have arrived yet. These
                // images will never be associated to poses.
                depth_buffer_.clear();
                rgb_buffer_.clear();      // OK to call even when RGB images are not used
                class_buffer_.clear();    // OK to call even when class images are not used
                instance_buffer_.clear(); // OK to call even when instance images are not used
                ROS_DEBUG_THROTTLE_NAMED(5, "matching", "Matching: waiting for pose");
                continue;
            }
            else {
                // Find the two closest poses and interpolate to the depth timestamp.
                geometry_msgs::TransformStamped prev_pose;
                geometry_msgs::TransformStamped next_pose;
                const InterpResult result =
                    get_surrounding_poses(pose_buffer_, depth_timestamp, prev_pose, next_pose);
                if (result == InterpResult::query_smaller) {
                    // Remove the depth image, it will never be matched to poses.
                    const std::lock_guard<std::mutex> depth_lock(depth_buffer_mutex_);
                    depth_buffer_.pop_front();
                    ROS_DEBUG_NAMED("matching", "Matching: depth timestamp earlier than all poses");
                    continue;
                }
                else if (result == InterpResult::query_greater) {
                    // Remove the first poses, they will never be matched to depth images.
                    pose_buffer_.erase_begin(pose_buffer_.size() - 1);
                    ROS_DEBUG_NAMED("matching", "Matching: depth timestamp later than all poses");
                    continue;
                }

                // Interpolate to associate a pose to the depth image.
                external_T_WC = interpolate_pose(prev_pose, next_pose, depth_timestamp);
            }
        }

        // Copy the semantics into the appropriate buffer.
        if (node_config_.enable_objects) {
            if (semantics_found) {
                input_segmentation_ =
                    to_supereight_segmentation(current_class_msg, current_instance_msg);
            }
            else {
                input_segmentation_ = se::SegmentationResult(0, 0);
            }
        }

        // The currect depth image is going to be integrated, remove it from the
        // buffer to avoid integrating it again.
        { // Block to reduce the scope of depth_lock.
            const std::lock_guard<std::mutex> depth_lock(depth_buffer_mutex_);
            depth_buffer_.pop_front();
        }
        end_time = std::chrono::steady_clock::now();

        if (node_config_.run_segmentation) {
            // If the network_mutex_ can be locked it means there is no thread running runNetwork().
            if (network_mutex_.try_lock()) {
                // Release the lock so that it can be acquired in runNetwork().
                network_mutex_.unlock();
                // Run the network in a background thread.
                std::thread t(std::bind(&SupereightNode::runNetwork,
                                        this,
                                        external_T_WC,
                                        current_depth_msg,
                                        current_rgb_msg,
                                        depth_timestamp));
                t.detach();
                // Don't call fuse with this depth frame as it will be done by runNetwork();
                continue;
            }
        }

        // Call fuse() if runNetwork() wasn't called.
        fuse(external_T_WC,
             current_depth_msg,
             current_rgb_msg,
             input_segmentation_,
             depth_timestamp);
    }
}



void SupereightNode::fuse(const Eigen::Matrix4f& T_WC,
                          const sensor_msgs::ImageConstPtr& depth_image,
                          const sensor_msgs::ImageConstPtr& color_image,
                          const se::SegmentationResult& segmentation,
                          const ros::Time& depth_timestamp,
                          const bool out_of_order_fusion)
{
    frame_++;
    const std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
    stats_.newFrame("fusion");
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::time_point<std::chrono::steady_clock> fusion_start_time =
        std::chrono::steady_clock::now();
    const int frame = frame_;

    ROS_INFO("-----------------------------------------");
    ROS_INFO("Frame %d", frame);

    // Convert the depth and RGB images into a format that supereight likes.
    to_supereight_depth(depth_image, sensor_.far_plane, input_depth_.get());
    if (node_config_.enable_rgb) {
        to_supereight_RGB(color_image, input_rgba_.get());
    }

    // Preprocessing
    start_time = std::chrono::steady_clock::now();
    pipeline_->preprocessDepth(
        input_depth_.get(), node_config_.input_res, supereight_config_.bilateral_filter);
    if (node_config_.enable_rgb) {
        pipeline_->preprocessColor(input_rgba_.get(), node_config_.input_res);
    }
    if (node_config_.enable_objects) {
        pipeline_->preprocessSegmentation(segmentation);
    }
    stats_.sample(
        "fusion",
        "Preprocessing",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());

    // Tracking
    start_time = std::chrono::steady_clock::now();
    bool tracked = false;
    if (node_config_.enable_tracking) {
        if (frame % supereight_config_.tracking_rate == 0) {
            tracked = pipeline_->track(sensor_, supereight_config_.icp_threshold);
        }
        else {
            tracked = false;
        }
    }
    else {
        pipeline_->setT_WC(T_WC);
        tracked = true;
    }
    // Call object tracking.
    if (node_config_.enable_objects) {
        pipeline_->trackObjects(sensor_, frame);
    }
    // Publish the pose estimated/received by supereight.
    const Eigen::Matrix4f se_T_WB = pipeline_->T_WC() * T_CB_;
    const Eigen::Vector3d se_t_WB = se_T_WB.block<3, 1>(0, 3).cast<double>();
    Eigen::Quaterniond se_q_WB(se_T_WB.block<3, 3>(0, 0).cast<double>());
    geometry_msgs::PoseStamped se_T_WB_msg;
    se_T_WB_msg.header = depth_image->header;
    se_T_WB_msg.header.frame_id = world_frame_id_;
    se_T_WB_msg.header.stamp = depth_timestamp;
    tf::pointEigenToMsg(se_t_WB, se_T_WB_msg.pose.position);
    tf::quaternionEigenToMsg(se_q_WB, se_T_WB_msg.pose.orientation);
    supereight_pose_pub_.publish(se_T_WB_msg);
    stats_.sample(
        "fusion",
        "Tracking",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());

    // Integration
    // Integrate only if tracking was successful or it is one of the first 4
    // frames.
    bool integrated = false;
    if ((tracked && (frame % supereight_config_.integration_rate == 0)) || frame <= 3) {
        {
            const std::lock_guard<std::mutex> map_lock(map_mutex_);

            start_time = std::chrono::steady_clock::now();
            integrated = pipeline_->integrate(sensor_, frame);
            stats_.sample(
                "fusion",
                "Integration",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)
                    .count());

            start_time = std::chrono::steady_clock::now();
            if (node_config_.enable_objects) {
                integrated = pipeline_->integrateObjects(sensor_, frame);
            }
            stats_.sample(
                "fusion",
                "Object integration",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)
                    .count());
        }

        if (!out_of_order_fusion) {
            const std::lock_guard<std::mutex> map_lock(pose_mutex_);
            planner_->setT_WB(se_T_WB, pipeline_->getDepth());
        }
    }
    else {
        integrated = false;
        stats_.sample("fusion", "Integration", 0.0);
        stats_.sample("fusion", "Object integration", 0.0);
    }

    // Rendering
    start_time = std::chrono::steady_clock::now();
    if (node_config_.enable_rendering) {
        // Depth
        pipeline_->renderDepth(depth_render_.get(), image_res_, sensor_);
        const sensor_msgs::Image depth_render_msg =
            RGBA_to_msg(depth_render_.get(), image_res_, depth_image->header);
        depth_render_pub_.publish(depth_render_msg);

        // RGB
        if (node_config_.enable_rgb) {
            pipeline_->renderRGBA(rgba_render_.get(), image_res_);
            const sensor_msgs::Image rgba_render_msg =
                RGBA_to_msg(rgba_render_.get(), image_res_, depth_image->header);
            rgba_render_pub_.publish(rgba_render_msg);
        }

        // Track
        if (node_config_.enable_tracking) {
            pipeline_->renderTrack(track_render_.get(), image_res_);
            const sensor_msgs::Image track_render_msg =
                RGBA_to_msg(track_render_.get(), image_res_, depth_image->header);
            track_render_pub_.publish(track_render_msg);
        }

        // Volume
        if (frame % supereight_config_.rendering_rate == 0) {
            (void) pipeline_->raycastObjectsAndBg(sensor_, frame);

            pipeline_->renderObjects(
                volume_render_.get(), image_res_, sensor_, RenderMode::InstanceID, false);
            volume_render_pub_.publish(
                RGBA_to_msg(volume_render_.get(), image_res_, depth_image->header));

            //pipeline_->renderObjects(
            //    volume_render_color_.get(), image_res_, sensor_, RenderMode::Color, false);
            pipeline_->renderVolume(volume_render_color_.get(), image_res_, sensor_, true);
            volume_render_color_pub_.publish(
                RGBA_to_msg(volume_render_color_.get(), image_res_, depth_image->header));

            pipeline_->renderObjects(
                volume_render_scale_.get(), image_res_, sensor_, RenderMode::Scale, false);
            volume_render_scale_pub_.publish(
                RGBA_to_msg(volume_render_scale_.get(), image_res_, depth_image->header));

            pipeline_->renderObjects(
                volume_render_min_scale_.get(), image_res_, sensor_, RenderMode::MinScale, false);
            volume_render_min_scale_pub_.publish(
                RGBA_to_msg(volume_render_min_scale_.get(), image_res_, depth_image->header));

            if (node_config_.enable_objects) {
                pipeline_->renderObjectClasses(class_render_.get(), image_res_);
                class_render_pub_.publish(
                    RGBA_to_msg(class_render_.get(), image_res_, depth_image->header));

                pipeline_->renderObjectInstances(instance_render_.get(), image_res_);
                instance_render_pub_.publish(
                    RGBA_to_msg(instance_render_.get(), image_res_, depth_image->header));
            }

            pipeline_->renderRaycast(raycast_render_.get(), image_res_);
            raycast_render_pub_.publish(
                RGBA_to_msg(raycast_render_.get(), image_res_, depth_image->header));
        }

        if (node_config_.visualize_360_raycasting) {
            // Entropy
            se::Image<uint32_t> entropy_render(1, 1);
            se::Image<uint32_t> depth_render(1, 1);
            if (supereight_config_.enable_exploration) {
                planner_->renderCurrentEntropyDepth(entropy_render, depth_render);
            }
            else {
                entropy_render = se::Image<uint32_t>(supereight_config_.raycast_width,
                                                     supereight_config_.raycast_height);
                depth_render = se::Image<uint32_t>(supereight_config_.raycast_width,
                                                   supereight_config_.raycast_height);
                render_pose_entropy_depth(entropy_render,
                                          depth_render,
                                          *(pipeline_->getMap()),
                                          sensor_,
                                          T_MW_ * pipeline_->T_WC() * T_CB_,
                                          supereight_config_.T_BC);
            }
            const sensor_msgs::Image entropy_render_msg =
                RGBA_to_msg(entropy_render.data(),
                            Eigen::Vector2i(entropy_render.width(), entropy_render.height()),
                            depth_image->header);
            entropy_render_pub_.publish(entropy_render_msg);
            const sensor_msgs::Image entropy_depth_render_msg =
                RGBA_to_msg(depth_render.data(),
                            Eigen::Vector2i(depth_render.width(), depth_render.height()),
                            depth_image->header);
            entropy_depth_render_pub_.publish(entropy_depth_render_msg);
        }
    }
    stats_.sample(
        "fusion",
        "Rendering",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());

    // Visualization
    start_time = std::chrono::steady_clock::now();
    map_pub_.publish(map_dim_msg_);
    visualizeEnvironmentAABB();
    visualizeSamplingAABB();
    if (node_config_.visualization_rate > 0 && (frame % node_config_.visualization_rate == 0)) {
        visualizeWholeMap();
        visualizeMapMesh();
        if (node_config_.enable_objects) {
            //visualizeObjects();
            visualizeObjectMeshes();
            visualizeObjectAABBs();
        }
        visualizeFrontiers();
        visualizePoseHistory();
        visualizePoseGridHistory();
    }
    stats_.sample(
        "fusion",
        "Visualization",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());

    stats_.sampleTimeNow("fusion", "Timestamp");
    stats_.sample("fusion", "Frame", frame);
    stats_.sample("fusion", "Free volume", pipeline_->free_volume);
    stats_.sample("fusion", "Occupied volume", pipeline_->occupied_volume);
    stats_.sample("fusion", "Explored volume", pipeline_->explored_volume);
    stats_.sample("fusion", "RAM usage", ram_usage_self() / 1024.0 / 1024.0);
    stats_.sample("fusion", "t_WB x", se_t_WB.x());
    stats_.sample("fusion", "t_WB y", se_t_WB.y());
    stats_.sample("fusion", "t_WB z", se_t_WB.z());
    stats_.sample("fusion", "q_WB x", se_q_WB.x());
    stats_.sample("fusion", "q_WB y", se_q_WB.y());
    stats_.sample("fusion", "q_WB z", se_q_WB.z());
    stats_.sample("fusion", "q_WB w", se_q_WB.w());
    stats_.sample("fusion", "Number of objects", pipeline_->getObjectMaps().size());

    if (supereight_config_.rendering_rate > 0
        && (frame + 1) % supereight_config_.rendering_rate == 0
        && supereight_config_.output_render_file != "") {
        stdfs::create_directories(supereight_config_.output_render_file);

        const int w = (pipeline_->getImageResolution()).x();
        const int h = (pipeline_->getImageResolution()).y();
        const std::string prefix = supereight_config_.output_render_file + "/";
        std::stringstream path_suffix_ss;
        path_suffix_ss << std::setw(5) << std::setfill('0') << frame << ".png";
        const std::string suffix = path_suffix_ss.str();

        std::unique_ptr<uint32_t[]> segmentation_render(new uint32_t[w * h]);
        pipeline_->renderInputSegmentation(segmentation_render.get(),
                                           pipeline_->getImageResolution());
        std::unique_ptr<uint32_t[]> volume_aabb_render(new uint32_t[w * h]);
        pipeline_->renderObjects(volume_aabb_render.get(),
                                 pipeline_->getImageResolution(),
                                 sensor_,
                                 RenderMode::InstanceID);

        lodepng_encode32_file(
            (prefix + "rgba_" + suffix).c_str(), (unsigned char*) rgba_render_.get(), w, h);
        lodepng_encode32_file(
            (prefix + "depth_" + suffix).c_str(), (unsigned char*) depth_render_.get(), w, h);
        lodepng_encode32_file(
            (prefix + "segm_" + suffix).c_str(), (unsigned char*) segmentation_render.get(), w, h);
        lodepng_encode32_file(
            (prefix + "volume_" + suffix).c_str(), (unsigned char*) volume_render_.get(), w, h);
        lodepng_encode32_file((prefix + "volume_color_" + suffix).c_str(),
                              (unsigned char*) volume_render_color_.get(),
                              w,
                              h);
        lodepng_encode32_file((prefix + "volume_scale_" + suffix).c_str(),
                              (unsigned char*) volume_render_scale_.get(),
                              w,
                              h);
        lodepng_encode32_file((prefix + "volume_min_scale_" + suffix).c_str(),
                              (unsigned char*) volume_render_min_scale_.get(),
                              w,
                              h);
        lodepng_encode32_file((prefix + "volume_aabb_" + suffix).c_str(),
                              (unsigned char*) volume_aabb_render.get(),
                              w,
                              h);
        lodepng_encode32_file(
            (prefix + "raycast_" + suffix).c_str(), (unsigned char*) raycast_render_.get(), w, h);

        //for (const auto& o : pipeline_->getObjectMaps()) {
        //    std::stringstream filename_ss;
        //    filename_ss << prefix << "aabb_mask_" << std::setw(5) << std::setfill('0') << frame
        //                << "_" << std::setw(3) << std::setfill('0') << o->instance_id << ".png";
        //    cv::imwrite(filename_ss.str(),
        //                o->bounding_volume_M_.raycastingMask(
        //                    pipeline_->getImageResolution(), pipeline_->T_MC(), sensor_));
        //}

        if (node_config_.enable_objects) {
            lodepng_encode32_file((prefix + "instance_" + suffix).c_str(),
                                  (unsigned char*) instance_render_.get(),
                                  w,
                                  h);
            lodepng_encode32_file(
                (prefix + "class_" + suffix).c_str(), (unsigned char*) class_render_.get(), w, h);
        }
        //std::stringstream history_dir_ss;
        //history_dir_ss << supereight_config_.output_render_file << "/history_" << std::setw(5)
        //               << std::setfill('0') << frame;
        //planner_->getPoseMaskHistory().writeMasks(history_dir_ss.str());
    }

    if (supereight_config_.meshing_rate > 0 && (frame + 1) % supereight_config_.meshing_rate == 0) {
        SupereightNode::saveMap();
    }

    stats_.sample(
        "fusion",
        "Total",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - fusion_start_time)
            .count());
    stats_.writeFrame("fusion");
    stats_.print("fusion",
                 {"Preprocessing",
                  "Tracking",
                  "Integration",
                  "Object integration",
                  "Rendering",
                  "Visualization",
                  "Total"});

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - exploration_start_time_)
            .count()
        > node_config_.max_exploration_time) {
        ROS_INFO("Reached time limit of %.3f s, stopping", node_config_.max_exploration_time);
        ros::shutdown();
    }
}



void SupereightNode::plan()
{
    ROS_DEBUG_NAMED("planning", "Planning: waiting for initial pose");
    // Wait for the first pose
    while (true) {
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        const std::lock_guard<std::mutex> pose_lock(pose_mutex_);
        if (!planner_->getT_WBHistory().empty()) {
            break;
        }
    }
    // Free the initial position to allow planning.
    ROS_DEBUG_NAMED("planning", "Planning: freeing the initial pose");
    {
        const std::lock_guard<std::mutex> map_lock(map_mutex_);
        pipeline_->freeInitialPosition(
            sensor_,
            ((node_config_.dataset == Dataset::Gazebo || node_config_.dataset == Dataset::Real)
                 ? "sphere"
                 : "cylinder"));
    }
    // Wait for a few frame to be integrated.
    ROS_DEBUG_NAMED("planning", "Planning: waiting for some integrations");
    while (frame_ < 3) {
        ros::Duration(0.5).sleep();
    }
    ROS_DEBUG_NAMED("planning", "Planning: starting");
    const std::string planning_log_dir = ros_log_dir() + "/latest/planning";
    // Exploration planning
    while ((num_failed_planning_iterations_ < max_failed_planning_iterations_
            || max_failed_planning_iterations_ == 0)
           && keep_running_) {
        bool goal_reached = false;

        // When using the srl controller, check the autopilot status to determine if the goal has been reached
        if (node_config_.control_interface == ControlInterface::SRL) {
            mav_interface_msgs::AutopilotStatusService srv;

            // Check that the autopilot is Initialised and in Idle mode, meaning that there are no remaining tasks in the
            // reference Queue
            if (mav_status_service_.call(srv)) {
                const bool autopilotInitialised = srv.response.status.initialised;
                const bool autopilotIdle = srv.response.status.taskType == srv.response.status.Idle;
                goal_reached = autopilotInitialised && autopilotIdle;

                std::string task_type;
                switch (srv.response.status.taskType) {
                case 0:
                    task_type = "Idle";
                    break;
                case 1:
                    task_type = "Waypoint";
                    break;
                case 2:
                    task_type = "Trajectory";
                    break;
                default:
                    task_type = "Invalid";
                }
                ROS_DEBUG_NAMED("autopilot",
                                "Autopilot status: %s %s",
                                autopilotInitialised ? "Initialized  " : "Uninitialized",
                                task_type.c_str());
            }
            else {
                goal_reached = false;
                ROS_DEBUG_NAMED("autopilot", "Autopilot status: Unknown");
            }
        }
        else {
            // We are not using the SRL controller
            const std::lock_guard<std::mutex> pose_lock(pose_mutex_);
            goal_reached = planner_->goalReached();
        }
        ROS_DEBUG_NAMED("goal", "Goal reached: %s", goal_reached ? "YES" : "NO");


        if (goal_reached || num_planning_iterations_ == 0) {
            visualizeCandidates(0.25f);
            visualizeGoal(0.25f);
            if (planner_->needsNewGoal()) {
                stats_.newFrame("planning");
                se::Path path_WB;
                {
                    const std::lock_guard<std::mutex> map_lock(map_mutex_);
                    const std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                    const auto start_time = std::chrono::steady_clock::now();
                    planner_->setPlanningT_WB(transform_to_eigen(pose_buffer_.back().transform)
                                              * T_CB_);
                    path_WB = planner_->computeNextPath_WB(pipeline_->getFrontiers(),
                                                           pipeline_->getObjectMaps());
                    const auto end_time = std::chrono::steady_clock::now();
                    stats_.sample("planning",
                                  "Planning time",
                                  std::chrono::duration<double>(end_time - start_time).count());
                }
                // Save and print statistics.
                stats_.sampleTimeNow("planning", "Timestamp");
                stats_.sample("planning", "Planning iteration", num_planning_iterations_);
                ROS_WARN("Planning iteration %d", num_planning_iterations_);
                ROS_WARN("%-25s %.5f s", "Planning", stats_.get("planning", "Planning time"));
                ROS_WARN("Candidates: %zu", planner_->candidateViews().size());
                ROS_WARN("Rejected candidates: %zu", planner_->rejectedCandidateViews().size());

                if (path_WB.empty()) {
                    num_failed_planning_iterations_++;
                }
                else {
                    visualizeCandidates(node_config_.dataset == Dataset::Real ? 0.1f : 1.0f);
                    visualizeGoal();
                    // Save more statistics.
                    const auto& goal_candidate =
                        planner_->candidateViews()[planner_->goalViewIndex()];
                    const Eigen::Matrix4f goal_T_WB = T_WM_ * goal_candidate.goalT_MB();
                    const Eigen::Vector3f goal_t_WB = goal_T_WB.topRightCorner<3, 1>();
                    const Eigen::Quaternionf goal_q_WB(goal_T_WB.topLeftCorner<3, 3>());
                    stats_.sample("planning", "Goal utility", goal_candidate.utility());
                    stats_.sample(
                        "planning", "Goal entropy utility", goal_candidate.entropyUtility());
                    stats_.sample(
                        "planning", "Goal object dist utility", goal_candidate.objectDistUtility());
                    stats_.sample(
                        "planning", "Goal bg dist utility", goal_candidate.bgDistUtility());
                    stats_.sample("planning", "Goal entropy gain", goal_candidate.entropy_gain_);
                    stats_.sample(
                        "planning", "Goal object dist gain", goal_candidate.object_dist_gain_);
                    stats_.sample("planning", "Goal bg dist gain", goal_candidate.bg_dist_gain_);
                    stats_.sample("planning", "Goal path time", goal_candidate.path_time_);
                    stats_.sample("planning",
                                  "Exploration dominant",
                                  static_cast<int>(planner_->explorationDominant()));
                    stats_.sample("planning", "Goal t_WB x", goal_t_WB.x());
                    stats_.sample("planning", "Goal t_WB y", goal_t_WB.y());
                    stats_.sample("planning", "Goal t_WB z", goal_t_WB.z());
                    stats_.sample("planning", "Goal q_WB x", goal_q_WB.x());
                    stats_.sample("planning", "Goal q_WB y", goal_q_WB.y());
                    stats_.sample("planning", "Goal q_WB z", goal_q_WB.z());
                    stats_.sample("planning", "Goal q_WB w", goal_q_WB.w());

                    ROS_DEBUG_NAMED("planning",
                                    "Planning %d, next goal is candidate %zu",
                                    num_planning_iterations_,
                                    planner_->goalViewIndex());
                    for (size_t i = 0; i < planner_->candidateViews().size(); ++i) {
                        ROS_DEBUG_NAMED("planning",
                                        "Planning %d candidate %2zu utility: %s",
                                        num_planning_iterations_,
                                        i,
                                        planner_->candidateViews()[i].utilityStr().c_str());
                    }
                    for (size_t i = 0; i < planner_->candidateViews().size(); ++i) {
                        const Eigen::Vector3f goal_t_WB =
                            (T_WM_
                             * planner_->candidateViews()[i].goalT_MB().topRightCorner<4, 1>())
                                .head<3>();
                        ROS_DEBUG_NAMED("planning",
                                        "Planning %d candidate %2zu t_WB: % .3f % .3f % .3f",
                                        num_planning_iterations_,
                                        i,
                                        goal_t_WB.x(),
                                        goal_t_WB.y(),
                                        goal_t_WB.z());
                    }
                    // Save candidate information.
                    //stdfs::create_directories(planning_log_dir);
                    //std::stringstream base_ss;
                    //base_ss << planning_log_dir << "/planning_" << std::setw(5) << std::setfill('0')
                    //        << num_planning_iterations_ << "_";
                    //const std::string base = base_ss.str();
                    //write_view_data(planner_->goalView(),
                    //                base + "goal_view.txt",
                    //                base + "goal_entropy.txt",
                    //                base + "goal_entropy.png",
                    //                base + "goal_depth.png",
                    //                base + "goal_object_dist_gain.png",
                    //                base + "goal_bg_dist_gain.png",
                    //                base + "goal_path_M.tsv");
                    //if (supereight_config_.output_mesh_file != "") {
                    //    saveCandidates();
                    //}
                }

                // Save (rejected) candidate information.
                //stdfs::create_directories(planning_log_dir);
                //std::stringstream base_ss;
                //base_ss << planning_log_dir << "/planning_" << std::setw(5) << std::setfill('0')
                //        << num_planning_iterations_ << "_";
                //const std::string base = base_ss.str();
                //for (size_t i = 0; i < planner_->candidateViews().size(); ++i) {
                //    std::stringstream prefix_ss;
                //    prefix_ss << "candidate_" << std::setw(2) << std::setfill('0') << i;
                //    const std::string prefix = prefix_ss.str();
                //    write_view_data(planner_->candidateViews()[i],
                //                    base + prefix + "_view.txt",
                //                    base + prefix + "_entropy.txt",
                //                    base + prefix + "_entropy.png",
                //                    base + prefix + "_depth.png",
                //                    base + prefix + "_object_dist_gain.png",
                //                    base + prefix + "_bg_dist_gain.png",
                //                    base + prefix + "_path_M.tsv");
                //}
                //for (size_t i = 0; i < planner_->rejectedCandidateViews().size(); ++i) {
                //    std::stringstream prefix_ss;
                //    prefix_ss << "rejected_" << std::setw(2) << std::setfill('0') << i;
                //    const std::string prefix = prefix_ss.str();
                //    write_view_data(planner_->rejectedCandidateViews()[i],
                //                    base + prefix + "_view.txt",
                //                    base + prefix + "_entropy.txt",
                //                    base + prefix + "_entropy.png",
                //                    base + prefix + "_depth.png",
                //                    base + prefix + "_object_dist_gain.png",
                //                    base + prefix + "_bg_dist_gain.png",
                //                    base + prefix + "_path_M.tsv");
                //}
                num_planning_iterations_++;
                stats_.writeFrame("planning");
            }
            // Change the path publishing method depending on the dataset type.
            if (node_config_.control_interface == ControlInterface::RotorS) {
                publish_path_open_loop(*planner_,
                                       path_pub_,
                                       world_frame_id_,
                                       node_config_.dataset,
                                       supereight_config_.delta_t);
            }
            else if (node_config_.control_interface == ControlInterface::SRL) {
                publish_full_state_trajectory(*planner_, path_pub_, supereight_config_);
            }
            else {
                publish_path_vertex(*planner_, path_pub_, world_frame_id_, node_config_.dataset);
            }
        }
        else if (num_planning_iterations_ > 0 && node_config_.dataset == Dataset::Real) {
            visualizeGoal();
            std::this_thread::sleep_for(std::chrono::duration<double>(0.05));
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(0.05));
    }
    ROS_INFO("Failed to plan %d times, stopping", num_failed_planning_iterations_);
    ros::shutdown();
}



void SupereightNode::saveMap()
{
    constexpr bool overwrite_old_meshes = false;

    if (supereight_config_.enable_meshing && !supereight_config_.output_mesh_file.empty()) {
        Eigen::Matrix4f T_HW = Eigen::Matrix4f::Identity();
        if (base_dataset(node_config_.dataset) == Dataset::Habitat) {
            geometry_msgs::TransformStamped tf;
            try {
                tf = tf_buffer_.lookupTransform("habitat", world_frame_id_, ros::Time(0));
                T_HW(0, 3) = tf.transform.translation.x;
                T_HW(1, 3) = tf.transform.translation.y;
                T_HW(2, 3) = tf.transform.translation.z;
                T_HW.topLeftCorner<3, 3>() = Eigen::Quaternionf(tf.transform.rotation.w,
                                                                tf.transform.rotation.x,
                                                                tf.transform.rotation.y,
                                                                tf.transform.rotation.z)
                                                 .toRotationMatrix();
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
        }

        std::stringstream output_mesh_dir_ss;
        output_mesh_dir_ss << supereight_config_.output_mesh_file;
        if constexpr (!overwrite_old_meshes) {
            output_mesh_dir_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
        }
        const std::string output_mesh_dir = output_mesh_dir_ss.str();
        stdfs::create_directories(output_mesh_dir);

        if (node_config_.dataset == Dataset::Real) {
            std::stringstream output_octomap_file_ss;
            output_octomap_file_ss << output_mesh_dir << "/mesh";
            if constexpr (!overwrite_old_meshes) {
                output_octomap_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
            }
            output_octomap_file_ss << ".bt";
            std::unique_ptr<octomap::OcTree> octomap(se::to_octomap(*(pipeline_->getMap())));
            if (octomap) {
                octomap->writeBinary(output_octomap_file_ss.str());
            }
        }

        if (node_config_.dataset == Dataset::Real) {
            const float z_min = (T_MW_ * supereight_config_.sampling_min_W.homogeneous()).z();
            const float z_max = (T_MW_ * supereight_config_.sampling_max_W.homogeneous()).z();
            const std::array z_values = {z_min, (z_min + z_max) / 2.0f, z_max};
            for (size_t i = 0; i < z_values.size(); ++i) {
                std::stringstream output_slice_voxel_file_ss;
                output_slice_voxel_file_ss << output_mesh_dir << "/slice";
                if constexpr (!overwrite_old_meshes) {
                    output_slice_voxel_file_ss << "_" << std::setw(5) << std::setfill('0')
                                               << frame_;
                }
                output_slice_voxel_file_ss << "_" << std::setw(1) << std::setfill('0') << i
                                           << ".vtk";
                pipeline_->saveThresholdSliceZ(output_slice_voxel_file_ss.str(), z_values[i]);
            }
        }

        {
            std::stringstream output_mesh_meter_file_ss;
            output_mesh_meter_file_ss << output_mesh_dir << "/mesh";
            if constexpr (!overwrite_old_meshes) {
                output_mesh_meter_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
            }
            output_mesh_meter_file_ss << ".ply";
            pipeline_->saveMesh(output_mesh_meter_file_ss.str(), T_HW);
        }

        {
            std::stringstream output_mesh_object_file_ss;
            output_mesh_object_file_ss << output_mesh_dir << "/mesh";
            if constexpr (!overwrite_old_meshes) {
                output_mesh_object_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
            }
            output_mesh_object_file_ss << "_object";
            pipeline_->saveObjectMeshes(output_mesh_object_file_ss.str(), T_HW);
        }

        {
            std::stringstream output_path_ply_file_ss;
            output_path_ply_file_ss << output_mesh_dir << "/path";
            if constexpr (!overwrite_old_meshes) {
                output_path_ply_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
            }
            output_path_ply_file_ss << ".ply";
            planner_->writePathPLY(output_path_ply_file_ss.str(), T_HW);
        }

        //{
        //    std::stringstream output_path_tsv_file_ss;
        //    output_path_tsv_file_ss << output_mesh_dir << "/path";
        //    if constexpr (!overwrite_old_meshes) {
        //        output_path_tsv_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
        //    }
        //    output_path_tsv_file_ss << ".tsv";
        //    planner_->writePathTSV(output_path_tsv_file_ss.str(), T_HW);
        //}

        if (node_config_.dataset == Dataset::Real) {
            std::stringstream output_wedges_ply_file_ss;
            output_wedges_ply_file_ss << output_mesh_dir << "/pose_grid_history";
            if constexpr (!overwrite_old_meshes) {
                output_wedges_ply_file_ss << "_" << std::setw(5) << std::setfill('0') << frame_;
            }
            output_wedges_ply_file_ss << ".ply";
            se::io::save_mesh_ply(planner_->getPoseGridHistory().wedgeMesh(),
                                  output_wedges_ply_file_ss.str(),
                                  T_HW * T_WM_);
        }
        ROS_INFO("Map saved in %s\n", output_mesh_dir.c_str());
    }
}



int SupereightNode::saveCandidates()
{
    if (supereight_config_.enable_meshing && !supereight_config_.output_mesh_file.empty()) {
        stdfs::create_directories(supereight_config_.output_mesh_file);
        std::stringstream output_candidate_mesh_file_ss;
        output_candidate_mesh_file_ss << supereight_config_.output_mesh_file << "/candidates_"
                                      << std::setw(5) << std::setfill('0')
                                      << num_planning_iterations_ << ".ply";
        const std::string filename = output_candidate_mesh_file_ss.str();
        // Open file
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_WARN("Unable to write file %s\n", filename.c_str());
            return 1;
        }
        // Write header
        const auto& candidates = planner_->candidateViews();
        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "comment Candidate view positions\n";
        file << "comment Generated by semanticeight\n";
        file << "element vertex " << candidates.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "property uchar red\n";
        file << "property uchar green\n";
        file << "property uchar blue\n";
        file << "end_header\n";
        // Write candidate positions
        for (const auto& candidate : candidates) {
            const Eigen::Vector3f goal_t_WB =
                (T_WM_ * candidate.goalT_MB().topRightCorner<4, 1>()).head<3>();
            file << goal_t_WB.x() << " " << goal_t_WB.y() << " " << goal_t_WB.z() << " 255 0 0\n";
        }
        file.close();
    }
    return 0;
}



void SupereightNode::readConfig(const ros::NodeHandle& nh_private)
{
    supereight_config_ = read_supereight_config(nh_private);
    ROS_INFO_STREAM(supereight_config_);

    node_config_ = read_supereight_node_config(nh_private);
    print_supereight_node_config(node_config_);
};



void SupereightNode::setupRos()
{
    // Initialize the constant messages
    map_dim_msg_ = mapDimMsg();

    // Pose subscriber
    if (!node_config_.enable_tracking) {
        if (node_config_.pose_topic_type == "geometry_msgs::PoseStamped") {
            pose_sub_ = nh_.subscribe(
                "/pose", node_config_.pose_buffer_size, &SupereightNode::poseStampedCallback, this);
        }
        else if (node_config_.pose_topic_type == "geometry_msgs::TransformStamped") {
            pose_sub_ = nh_.subscribe("/pose",
                                      node_config_.pose_buffer_size,
                                      &SupereightNode::transformStampedCallback,
                                      this);
        }
        else {
            ROS_FATAL("Invalid pose topic type %s", node_config_.pose_topic_type.c_str());
            ROS_FATAL("Expected geometry_msgs::PoseStamped or geometry_msgs::TransformStamped");
            abort();
        }
    }
    // Depth subscriber
    depth_sub_ = nh_.subscribe("/camera/depth_image",
                               node_config_.depth_buffer_size,
                               &SupereightNode::depthCallback,
                               this);
    // RGB subscriber
    if (node_config_.enable_rgb) {
        rgb_sub_ = nh_.subscribe(
            "/camera/rgb_image", node_config_.rgb_buffer_size, &SupereightNode::RGBCallback, this);
        if (!node_config_.run_segmentation && node_config_.enable_objects) {
            class_sub_ = nh_.subscribe("/camera/class",
                                       node_config_.rgb_buffer_size,
                                       &SupereightNode::SemClassCallback,
                                       this);
            instance_sub_ = nh_.subscribe("/camera/instance",
                                          node_config_.rgb_buffer_size,
                                          &SupereightNode::SemInstanceCallback,
                                          this);
        }
    }

    // Publishers
    supereight_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/supereight/pose",
                                                                     node_config_.pose_buffer_size);

    // Initialise the reference publisher depending on the simulator and controller used
    if (node_config_.control_interface == ControlInterface::RotorS) {
        path_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/supereight/path", 2);
    }
    else if (node_config_.control_interface == ControlInterface::SRL) {
        path_pub_ = nh_.advertise<mav_interface_msgs::FullStateTrajectory>("/supereight/path", 2);
    }
    else {
        path_pub_ = nh_.advertise<nav_msgs::Path>("/supereight/path", 2);
    }

    static_tf_broadcaster_.sendTransform(T_WM_Msg());
    static_tf_broadcaster_.sendTransform(T_BC_Msg());
    // Render publishers
    if (node_config_.enable_rendering) {
        depth_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/depth_render", 1);
        if (node_config_.enable_rgb) {
            rgba_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/rgba_render", 1);
        }
        if (node_config_.enable_tracking) {
            track_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/track_render", 1);
        }
        volume_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/volume_render", 1);
        volume_render_color_pub_ =
            nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_color", 1);
        volume_render_scale_pub_ =
            nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_scale", 1);
        volume_render_min_scale_pub_ =
            nh_.advertise<sensor_msgs::Image>("/supereight/volume_render_min_scale", 1);
        class_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/class_render", 1);
        instance_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/instance_render", 1);
        raycast_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/raycast_render", 1);
        entropy_render_pub_ = nh_.advertise<sensor_msgs::Image>("/supereight/entropy_render", 1);
        entropy_depth_render_pub_ =
            nh_.advertise<sensor_msgs::Image>("/supereight/entropy_depth_render", 1);
    }

    // Visualization publishers
    map_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map", 4);
    object_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/objects", 10);
    map_frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/map/frontiers", 10);
    map_candidate_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/supereight/planner/candidates", 100);
    map_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/planner/goal", 10);
    mav_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/mav", 1);
    pose_history_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/supereight/planner/pose_history", 2);
    limit_pub_ = nh_.advertise<visualization_msgs::Marker>("/supereight/limits", 2);

    // ROS services. // ToDo -> use default namespace and bind everything in the launch file
    mav_status_service_ =
        nh_.serviceClient<mav_interface_msgs::AutopilotStatusService>("/autopilot/statusService");
}



void SupereightNode::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    const std::lock_guard<std::mutex> depth_lock(depth_buffer_mutex_);
    depth_buffer_.push_back(depth_msg);
    ROS_DEBUG_NAMED(
        "buffers", "Depth image buffer: %lu/%lu", depth_buffer_.size(), depth_buffer_.capacity());
}



void SupereightNode::RGBCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
    const std::lock_guard<std::mutex> rgb_lock(rgb_buffer_mutex_);
    rgb_buffer_.push_back(rgb_msg);
    ROS_DEBUG_NAMED(
        "buffers", "RGB image buffer:   %lu/%lu", rgb_buffer_.size(), rgb_buffer_.capacity());
}



void SupereightNode::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& T_WB_msg)
{
    // Convert the message to an Eigen matrix.
    Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_WB;
    tf::quaternionMsgToEigen(T_WB_msg->pose.orientation, q_WB);
    T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
    Eigen::Vector3d t_WB;
    tf::pointMsgToEigen(T_WB_msg->pose.position, t_WB);
    T_WB.topRightCorner<3, 1>() = t_WB;

    if (node_config_.center_at_first_position) {
        if (std::isnan(init_t_WB_.x())) {
            // This is the first pose.
            init_t_WB_ = T_WB.topRightCorner<3, 1>().cast<float>();
        }
        // Subtract the initial position.
        T_WB.topRightCorner<3, 1>() -= init_t_WB_.cast<double>();
    }

    // Call the generic pose callback.
    poseCallback(T_WB, T_WB_msg->header);
}



void SupereightNode::transformStampedCallback(
    const geometry_msgs::TransformStamped::ConstPtr& T_WB_msg)
{
    // Convert the message to an Eigen matrix.
    Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_WB;
    tf::quaternionMsgToEigen(T_WB_msg->transform.rotation, q_WB);
    T_WB.topLeftCorner<3, 3>() = q_WB.toRotationMatrix();
    Eigen::Vector3d t_WB;
    tf::vectorMsgToEigen(T_WB_msg->transform.translation, t_WB);
    T_WB.topRightCorner<3, 1>() = t_WB;

    if (node_config_.center_at_first_position) {
        if (std::isnan(init_t_WB_.x())) {
            // This is the first pose.
            init_t_WB_ = T_WB.topRightCorner<3, 1>().cast<float>();
        }
        // Subtract the initial position.
        T_WB.topRightCorner<3, 1>() -= init_t_WB_.cast<double>();
    }

    // Call the generic pose callback.
    poseCallback(T_WB, T_WB_msg->header);
}



void SupereightNode::poseCallback(const Eigen::Matrix4d& T_WB, const std_msgs::Header& header)
{
    // Convert body pose to camera pose.
    const Eigen::Matrix4d T_WC = T_WB * supereight_config_.T_BC.cast<double>();

    // Create a ROS message from T_WC.
    geometry_msgs::TransformStamped T_WC_msg;
    T_WC_msg.header = header;
    T_WC_msg.header.frame_id = world_frame_id_;
    const Eigen::Quaterniond q_WC(T_WC.topLeftCorner<3, 3>());
    tf::quaternionEigenToMsg(q_WC, T_WC_msg.transform.rotation);
    const Eigen::Vector3d t_WC = T_WC.topRightCorner<3, 1>();
    tf::vectorEigenToMsg(t_WC, T_WC_msg.transform.translation);

    {
        // Put it into the buffer.
        const std::lock_guard<std::mutex> pose_lock(pose_buffer_mutex_);
        pose_buffer_.push_back(T_WC_msg);
        ROS_DEBUG_NAMED(
            "buffers", "Pose buffer:        %lu/%lu", pose_buffer_.size(), pose_buffer_.capacity());
    }

    // Update the Body-World transform. In the case of Gazebo/Real a frame with the same name is
    // already being published.
    if (node_config_.dataset != Dataset::Gazebo && node_config_.dataset != Dataset::Real) {
        geometry_msgs::TransformStamped T_WB_msg;
        T_WB_msg.header = header;
        T_WB_msg.header.frame_id = world_frame_id_;
        T_WB_msg.child_frame_id = body_frame_id_;
        const Eigen::Quaterniond q_WB(T_WB.topLeftCorner<3, 3>());
        tf::quaternionEigenToMsg(q_WB, T_WB_msg.transform.rotation);
        const Eigen::Vector3d t_WB = T_WB.topRightCorner<3, 1>();
        tf::vectorEigenToMsg(t_WB, T_WB_msg.transform.translation);
        pose_tf_broadcaster_.sendTransform(T_WB_msg);
    }

    visualizeMAV();
}



void SupereightNode::SemClassCallback(const sensor_msgs::ImageConstPtr& class_msg)
{
    const std::lock_guard<std::mutex> class_lock(class_buffer_mutex_);
    class_buffer_.push_back(class_msg);
    ROS_DEBUG_NAMED(
        "buffers", "Class image buffer: %lu/%lu", class_buffer_.size(), class_buffer_.capacity());
}



void SupereightNode::SemInstanceCallback(const sensor_msgs::ImageConstPtr& instance_msg)
{
    const std::lock_guard<std::mutex> instance_lock(instance_buffer_mutex_);
    instance_buffer_.push_back(instance_msg);
    ROS_DEBUG_NAMED("buffers",
                    "Inst. image buffer: %lu/%lu",
                    instance_buffer_.size(),
                    instance_buffer_.capacity());
}



geometry_msgs::TransformStamped SupereightNode::T_WM_Msg() const
{
    // Transform from world frame to map frame. ROS probably uses a different
    // convention than us?
    static geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = world_frame_id_;
    tf.child_frame_id = map_frame_id_;
    tf::vectorEigenToMsg(T_WM_.topRightCorner<3, 1>().cast<double>(), tf.transform.translation);
    tf.transform.rotation = make_quaternion();
    return tf;
}



geometry_msgs::TransformStamped SupereightNode::T_BC_Msg() const
{
    // Transform from camera frame to body frame. ROS probably uses a different
    // convention than us?
    static geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = body_frame_id_;
    tf.child_frame_id = camera_frame_id_;
    const Eigen::Matrix4d T_BC = supereight_config_.T_BC.cast<double>();
    const Eigen::Quaterniond q_BC(T_BC.topLeftCorner<3, 3>());
    tf::quaternionEigenToMsg(q_BC, tf.transform.rotation);
    const Eigen::Vector3d t_BC = T_BC.topRightCorner<3, 1>();
    tf::vectorEigenToMsg(t_BC, tf.transform.translation);
    return tf;
}



visualization_msgs::Marker SupereightNode::mapDimMsg() const
{
    static visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = map_frame_id_;
    m.ns = "dim";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = make_point(supereight_config_.map_dim.x() / 2.0f);
    m.pose.orientation = make_quaternion();
    m.scale = make_vector3(supereight_config_.map_dim.x());
    m.color = make_color(1.0f, 1.0f, 1.0f, 0.1f);
    m.lifetime = ros::Duration(0.0);
    m.frame_locked = true;
    return m;
}



void SupereightNode::runNetwork(const Eigen::Matrix4f& T_WC,
                                const sensor_msgs::ImageConstPtr& depth_image,
                                const sensor_msgs::ImageConstPtr& color_image,
                                const ros::Time& depth_timestamp)
{
    // runNetwork() should only be run by a single thread at a time so that matchAndFuse() can fall
    // back to calling fuse(). Return if the lock can't be acquired (another thread is already
    // running).
    std::unique_lock<std::mutex> network_lock(network_mutex_, std::defer_lock_t());
    if (!network_lock.try_lock()) {
        return;
    }
    stats_.newFrame("network");

    std::chrono::time_point<std::chrono::steady_clock> start_time =
        std::chrono::steady_clock::now();

    cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(color_image, "rgb8");
    se::SegmentationResult segmentation(rgb_ptr->image.cols, rgb_ptr->image.rows);
#ifdef SE_WITH_MASKRCNN
    const std::vector<mr::Detection> detections = network_.infer(rgb_ptr->image, false);
    //cv::imwrite("/home/srl/frame_" + std::to_string(frame_) + ".png",
    //    mr::visualize_detections(detections, rgb_ptr->image));
    // Convert the object detections to a SegmentationResult.
    for (const auto& d : detections) {
        segmentation.object_instances.emplace_back(
            se::instance_new, se::DetectionConfidence(d.class_id, d.confidence), d.mask);
    }
#endif // SE_WITH_MASKRCNN

    std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
    stats_.sampleTimeNow("network", "Timestamp");
    stats_.sample(
        "network", "Network time", std::chrono::duration<double>(end_time - start_time).count());
    stats_.writeFrame("network");
    ROS_INFO("%-25s %.5f s", "Network", stats_.get("network", "Network time"));

    fuse(T_WC, depth_image, color_image, segmentation, depth_timestamp, true);
}

} // namespace se
