// SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2019 Anna Dai
// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <csignal>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <functional>
#include <lodepng.h>
#include <map>
#include <thread>

#include "se/voxel_implementations.hpp"
#include "supereight_ros/filesystem.hpp"
#include "supereight_ros/supereight_ros.hpp"
#include "supereight_ros/utilities.hpp"



auto get_image_timestamp = [](sensor_msgs::ImageConstPtr img) { return img->header.stamp; };

namespace se {
void SupereightNode::visualizeWholeMap()
{
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;
    // Iterate over the octree, creating a different CUBE_LIST marker for each
    // volume size and state.
    std::map<int, visualization_msgs::Marker> markers_free;
    std::map<int, visualization_msgs::Marker> markers_occupied;
    std::map<int, visualization_msgs::Marker> markers_unknown;
    for (const auto& volume : *(pipeline_->getMap())) {
        // Select a marker list and color depending on the volume state.
        std::map<int, visualization_msgs::Marker>* markers = nullptr;
        std::string ns;
        std_msgs::ColorRGBA volume_color;
        if (is_free<VoxelImpl>(volume)) {
            markers = &markers_free;
            ns = "free_voxels";
            volume_color = eigen_to_color(color_free_);
        }
        else if (is_occupied<VoxelImpl>(volume)) {
            markers = &markers_occupied;
            ns = "occupied_voxels";
            volume_color = eigen_to_color(color_occupied_);
        }
        else {
            markers = &markers_unknown;
            ns = "unknown_voxels";
            volume_color = eigen_to_color(color_unknown_);
        }
        const int size = volume.size;
        if (markers->count(size) == 0) {
            // Initialize the Marker message for this cube size.
            (*markers)[size] = visualization_msgs::Marker();
            (*markers)[size].header = header;
            (*markers)[size].ns = ns;
            (*markers)[size].id = size;
            (*markers)[size].type = visualization_msgs::Marker::CUBE_LIST;
            (*markers)[size].action = visualization_msgs::Marker::ADD;
            (*markers)[size].pose.orientation = make_quaternion();
            (*markers)[size].scale = make_vector3(volume.dim);
            (*markers)[size].color = volume_color;
            (*markers)[size].lifetime = ros::Duration(0.0);
            (*markers)[size].frame_locked = true;
        }
        // Append the current volume.
        const float voxel_top_height_W = volume.centre_M.z() + volume.dim / 2.0f - T_MW_(2, 3);
        if (voxel_top_height_W <= node_config_.visualization_max_z) {
            (*markers)[size].points.push_back(eigen_to_point(volume.centre_M));
        }
    }
    // Publish all markers.
    for (const auto& marker : markers_free) {
        map_pub_.publish(marker.second);
    }
    for (const auto& marker : markers_occupied) {
        map_pub_.publish(marker.second);
    }
    for (const auto& marker : markers_unknown) {
        map_pub_.publish(marker.second);
    }
}



void SupereightNode::visualizeMapMesh()
{
    const std::vector<se::Triangle> mesh = pipeline_->triangleMeshV();
    const float voxel_dim = pipeline_->getMap()->voxelDim();
    const Eigen::Matrix4f T_WM = pipeline_->T_WM();
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = world_frame_id_;
    visualization_msgs::Marker mesh_marker;
    mesh_marker = visualization_msgs::Marker();
    mesh_marker.header = header;
    mesh_marker.ns = "mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose.position = make_point();
    mesh_marker.pose.orientation = make_quaternion();
    mesh_marker.scale = make_vector3(1.0);
    mesh_marker.color = eigen_to_color(color_occupied_);
    mesh_marker.lifetime = ros::Duration(0.0);
    mesh_marker.frame_locked = true;
    mesh_marker.points.resize(3 * mesh.size());
    for (size_t i = 0; i < mesh.size(); i++) {
        const auto& triangle = mesh[i];
        // Convert the triangle vertices from the octree frame to the world frame.
        const Eigen::Vector3f v0_W =
            (T_WM * (voxel_dim * triangle.vertexes[0]).homogeneous()).head<3>();
        const Eigen::Vector3f v1_W =
            (T_WM * (voxel_dim * triangle.vertexes[1]).homogeneous()).head<3>();
        const Eigen::Vector3f v2_W =
            (T_WM * (voxel_dim * triangle.vertexes[2]).homogeneous()).head<3>();
        const float triangle_max_height_W =
            Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
        // Skip triangles above the height threshold.
        if (triangle_max_height_W <= node_config_.visualization_max_z) {
            mesh_marker.points[3 * i + 0] = eigen_to_point(v0_W);
            mesh_marker.points[3 * i + 1] = eigen_to_point(v1_W);
            mesh_marker.points[3 * i + 2] = eigen_to_point(v2_W);
        }
    }
    map_pub_.publish(mesh_marker);
}



void SupereightNode::visualizeObjects()
{
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;
    // Publish the object maps
    const Objects objects = pipeline_->getObjectMaps();
    std::map<int, visualization_msgs::Marker> markers_objects;
    for (size_t i = 0; i < objects.size(); ++i) {
        for (const auto& volume : *(objects[i]->map_)) {
            if (is_occupied<MultiresTSDF>(volume)) {
                const int size = volume.size;
                if (markers_objects.count(size) == 0) {
                    // Initialize the Marker message for this cube size.
                    markers_objects[size] = visualization_msgs::Marker();
                    markers_objects[size].header = header;
                    markers_objects[size].ns = "voxels";
                    markers_objects[size].id = size;
                    markers_objects[size].type = visualization_msgs::Marker::CUBE_LIST;
                    markers_objects[size].action = visualization_msgs::Marker::ADD;
                    markers_objects[size].pose.orientation = make_quaternion();
                    markers_objects[size].scale = make_vector3(volume.dim);
                    markers_objects[size].lifetime = ros::Duration(0.0);
                    markers_objects[size].frame_locked = true;
                }
                // Set the cube centre
                // centre_M is actually centre_O
                const Eigen::Vector3f centre_M =
                    (objects[i]->T_MO_ * volume.centre_M.homogeneous()).head<3>();
                const float voxel_top_height_W = centre_M.z() + volume.dim / 2.0f - T_MW_(2, 3);
                if (voxel_top_height_W <= node_config_.visualization_max_z) {
                    markers_objects[size].points.push_back(eigen_to_point(centre_M));
                    // Set the cube color based on the instance ID
                    const int instance_id = objects[i]->instance_id;
                    const Eigen::Vector3f c =
                        se::internal::color_map[instance_id % se::internal::color_map.size()]
                        / 255.0f;
                    markers_objects[size].colors.push_back(eigen_to_color(c.homogeneous()));
                }
            }
        }
    }
    for (const auto& marker : markers_objects) {
        object_pub_.publish(marker.second);
    }
}



void SupereightNode::visualizeObjectMeshes()
{
    const Eigen::Matrix4f T_WM = pipeline_->T_WM();
    const Objects& objects = pipeline_->getObjectMaps();
    const std::vector<std::vector<se::Triangle>> meshes =
        pipeline_->objectTriangleMeshesV(se::meshing::ScaleMode::Scale3);
    if (objects.size() != meshes.size()) {
        ROS_FATAL("Got different numbers of objects and object meshes.");
        abort();
    }
    // Publish each object mesh as a separate message.
    for (size_t i = 0; i < objects.size(); ++i) {
        const int instance_id = objects[i]->instance_id;
        const float voxel_dim = objects[i]->voxelDim();
        const Eigen::Matrix4f T_MO = objects[i]->T_MO_;
        const Eigen::Vector3f c =
            se::internal::color_map[instance_id % se::internal::color_map.size()] / 255.0f;
        visualization_msgs::Marker mesh_marker;
        mesh_marker.header.stamp = ros::Time::now();
        mesh_marker.header.frame_id = world_frame_id_;
        mesh_marker.ns = "meshes";
        mesh_marker.id = objects[i]->instance_id;
        mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        mesh_marker.action = visualization_msgs::Marker::ADD;
        mesh_marker.pose.position = make_point();
        mesh_marker.pose.orientation = make_quaternion();
        mesh_marker.scale = make_vector3(1.0);
        mesh_marker.color = eigen_to_color(c.homogeneous());
        mesh_marker.lifetime = ros::Duration(0.0);
        mesh_marker.frame_locked = true;
        mesh_marker.points.resize(3 * meshes[i].size());
        for (size_t j = 0; j < meshes[i].size(); j++) {
            const auto& triangle = meshes[i][j];
            // Convert the triangle vertices from the octree frame to the world frame.
            const Eigen::Vector3f v0_W =
                (T_WM * T_MO * (voxel_dim * triangle.vertexes[0]).homogeneous()).head<3>();
            const Eigen::Vector3f v1_W =
                (T_WM * T_MO * (voxel_dim * triangle.vertexes[1]).homogeneous()).head<3>();
            const Eigen::Vector3f v2_W =
                (T_WM * T_MO * (voxel_dim * triangle.vertexes[2]).homogeneous()).head<3>();
            mesh_marker.points[3 * j + 0] = eigen_to_point(v0_W);
            mesh_marker.points[3 * j + 1] = eigen_to_point(v1_W);
            mesh_marker.points[3 * j + 2] = eigen_to_point(v2_W);
        }
        object_pub_.publish(mesh_marker);
    }
}



void SupereightNode::visualizeObjectAABBs()
{
    for (const auto& o : pipeline_->getObjectMaps()) {
        const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> edges =
            o->bounding_volume_M_.edges();

        visualization_msgs::Marker aabb_marker;
        aabb_marker.header.stamp = ros::Time::now();
        aabb_marker.header.frame_id = map_frame_id_;
        aabb_marker.ns = "aabbs";
        aabb_marker.id = o->instance_id;
        aabb_marker.type = visualization_msgs::Marker::LINE_LIST;
        aabb_marker.action = visualization_msgs::Marker::ADD;
        aabb_marker.pose.position = make_point();
        aabb_marker.pose.orientation = make_quaternion();
        aabb_marker.scale.x = 0.05;
        aabb_marker.color = eigen_to_color(color_pose_history_);
        aabb_marker.lifetime = ros::Duration(0.0);
        aabb_marker.frame_locked = true;
        aabb_marker.points.resize(edges.size());
        for (size_t i = 0; i < edges.size(); i++) {
            aabb_marker.points[i] = eigen_to_point(edges[i]);
        }

        object_pub_.publish(aabb_marker);
    }
}



void SupereightNode::visualizeFrontiers()
{
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;
    // Publish the frontiers
    std::map<int, visualization_msgs::Marker> markers_frontiers;
    for (const auto& volume : pipeline_->frontierVolumes()) {
        const int size = volume.size;
        if (markers_frontiers.count(size) == 0) {
            // Initialize the Marker message for this cube size.
            std_msgs::ColorRGBA volume_color;
            volume_color = eigen_to_color(color_frontier_);
            markers_frontiers[size] = visualization_msgs::Marker();
            markers_frontiers[size].header = header;
            markers_frontiers[size].ns = "frontiers";
            markers_frontiers[size].id = size;
            markers_frontiers[size].type = visualization_msgs::Marker::CUBE_LIST;
            markers_frontiers[size].action = visualization_msgs::Marker::ADD;
            markers_frontiers[size].pose.orientation = make_quaternion();
            markers_frontiers[size].scale = make_vector3(volume.dim);
            markers_frontiers[size].color = volume_color;
            markers_frontiers[size].lifetime = ros::Duration(0.0);
            markers_frontiers[size].frame_locked = true;
        }
        // Append the current volume.
        const float voxel_top_height_W = volume.centre_M.z() + volume.dim / 2.0f - T_MW_(2, 3);
        if (voxel_top_height_W <= node_config_.visualization_max_z) {
            markers_frontiers[size].points.push_back(eigen_to_point(volume.centre_M));
        }
    }
    for (const auto& marker : markers_frontiers) {
        map_frontier_pub_.publish(marker.second);
    }
}



void SupereightNode::visualizeCandidates(float opacity)
{
    hideCandidates();
    const float diameter = 2.0f * supereight_config_.robot_radius;
    // Initialize the message header.
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;
    // Visualize the candidate poses as arrows.
    visualization_msgs::Marker pose_marker;
    pose_marker.header = header;
    pose_marker.ns = "poses";
    pose_marker.id = 0;
    pose_marker.type = visualization_msgs::Marker::ARROW;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.scale = make_vector3(0.5f, 0.05f, 0.05f);
    pose_marker.color = eigen_to_color(color_candidate_);
    if (opacity >= 0.0f) {
        pose_marker.color.a = opacity;
    }
    pose_marker.lifetime = ros::Duration(0.0);
    pose_marker.frame_locked = true;
    // Visualize the candidate IDs as text labels.
    visualization_msgs::Marker id_marker;
    id_marker.header = header;
    id_marker.ns = "ids";
    id_marker.id = 0;
    id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    id_marker.action = visualization_msgs::Marker::ADD;
    id_marker.pose.orientation = make_quaternion();
    id_marker.scale.z = 0.5f;
    id_marker.color = eigen_to_color(color_candidate_);
    if (opacity >= 0.0f) {
        id_marker.color.a = opacity;
    }
    id_marker.lifetime = ros::Duration(0.0);
    id_marker.frame_locked = true;
    // Visualize the candidate paths as line strips.
    visualization_msgs::Marker path_marker;
    path_marker.header = header;
    path_marker.ns = "paths";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation = make_quaternion();
    path_marker.scale.x = 0.05f;
    path_marker.color = eigen_to_color(color_candidate_);
    if (opacity >= 0.0f) {
        path_marker.color.a = opacity;
    }
    path_marker.lifetime = ros::Duration(0.0);
    path_marker.frame_locked = true;
    // Visualize the candidate desired positions as spheres.
    visualization_msgs::Marker desired_marker;
    desired_marker.header = header;
    desired_marker.ns = "desired_positions";
    desired_marker.id = 0;
    desired_marker.type = visualization_msgs::Marker::SPHERE;
    desired_marker.action = visualization_msgs::Marker::ADD;
    desired_marker.pose.orientation = make_quaternion();
    desired_marker.scale = make_vector3(diameter);
    desired_marker.color = eigen_to_color(color_candidate_);
    desired_marker.color.a = node_config_.dataset == Dataset::Real ? 0.05f : 0.25f;
    desired_marker.lifetime = ros::Duration(0.0);
    desired_marker.frame_locked = true;
    // Publish the markers for each candidate.
    for (const auto& candidate : planner_->candidateViews()) {
        if (candidate.isValid()) {
            // Pose.
            const Eigen::Matrix4f& goal_T_MB = candidate.goalT_MB();
            pose_marker.pose.position = eigen_to_point(goal_T_MB.topRightCorner<3, 1>());
            pose_marker.pose.orientation =
                eigen_to_quaternion(Eigen::Quaternionf(goal_T_MB.topLeftCorner<3, 3>()));
            map_candidate_pub_.publish(pose_marker);
            pose_marker.id++;
            // ID.
            if (node_config_.dataset != Dataset::Real) {
                id_marker.pose.position = pose_marker.pose.position;
                // Place the ID above the candidate
                id_marker.pose.position.z += id_marker.scale.z / 2.0f;
                id_marker.text = std::to_string(id_marker.id);
                map_candidate_pub_.publish(id_marker);
                id_marker.id++;
            }
            // Path.
            path_marker.points.clear();
            const se::Path& path = candidate.path();
            if (path.size() > 1) {
                for (const auto& T_MB : path) {
                    path_marker.points.push_back(eigen_to_point(T_MB.topRightCorner<3, 1>()));
                }
            }
            map_candidate_pub_.publish(path_marker);
            path_marker.id++;
            // Desired position.
            desired_marker.pose.position = eigen_to_point(candidate.desired_t_MB_);
            map_candidate_pub_.publish(desired_marker);
            desired_marker.id++;
        }
    }
    if (node_config_.dataset != Dataset::Real) {
        // Visualize the rejected candidate positions as spheres.
        visualization_msgs::Marker rejected_marker;
        rejected_marker.header = header;
        rejected_marker.ns = "rejected";
        rejected_marker.id = 0;
        rejected_marker.type = visualization_msgs::Marker::SPHERE;
        rejected_marker.action = visualization_msgs::Marker::ADD;
        rejected_marker.pose.orientation = make_quaternion();
        rejected_marker.scale = make_vector3(diameter);
        rejected_marker.color = eigen_to_color(color_rejected_candidate_);
        rejected_marker.lifetime = ros::Duration(0.0);
        rejected_marker.frame_locked = true;
        // Publish the marker for each rejected candidate.
        for (const auto& candidate : planner_->rejectedCandidateViews()) {
            const Eigen::Matrix4f& goal_T_MB = candidate.goalT_MB();
            rejected_marker.pose.position = eigen_to_point(goal_T_MB.topRightCorner<3, 1>());
            map_candidate_pub_.publish(rejected_marker);
            rejected_marker.id++;
        }
    }
}



void SupereightNode::hideCandidates()
{
    visualization_msgs::Marker del_marker;
    del_marker.action = visualization_msgs::Marker::DELETEALL;
    map_candidate_pub_.publish(del_marker);
}



void SupereightNode::visualizeGoal(float opacity)
{
    hideGoal();
    const se::CandidateView& goal_view = planner_->goalView();
    if (!goal_view.isValid()) {
        return;
    }
    const se::Path& path = goal_view.path();
    // Initialize the message header.
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;

    // Visualize the goal path poses as arrows.
    visualization_msgs::Marker pose_marker;
    pose_marker.header = header;
    pose_marker.ns = "poses";
    pose_marker.id = 0;
    pose_marker.type = visualization_msgs::Marker::ARROW;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.scale = make_vector3(0.5f, 0.05f, 0.05f);
    pose_marker.color = eigen_to_color(color_goal_);
    if (opacity >= 0.0f) {
        pose_marker.color.a = opacity;
    }
    pose_marker.lifetime = ros::Duration(0.0);
    pose_marker.frame_locked = true;
    // Publish an arrow for each path vertex.
    for (const Eigen::Matrix4f& T_MB : path) {
        pose_marker.pose.position = eigen_to_point(T_MB.topRightCorner<3, 1>());
        pose_marker.pose.orientation =
            eigen_to_quaternion(Eigen::Quaternionf(T_MB.topLeftCorner<3, 3>()));
        map_goal_pub_.publish(pose_marker);
        // Only the final arrow marker appears without the sleep.
        std::this_thread::sleep_for(std::chrono::duration<double>(0.00001));
        pose_marker.id++;
    }

    // Visualize the goal path as line strips.
    if (path.size() > 1) {
        visualization_msgs::Marker path_marker;
        path_marker.header = header;
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation = make_quaternion();
        path_marker.scale.x = 0.05f;
        path_marker.color = eigen_to_color(color_goal_);
        if (opacity >= 0.0f) {
            path_marker.color.a = opacity;
        }
        path_marker.lifetime = ros::Duration(0.0);
        path_marker.frame_locked = true;
        for (const auto& T_MB : path) {
            path_marker.points.push_back(eigen_to_point(T_MB.topRightCorner<3, 1>()));
        }
        map_goal_pub_.publish(path_marker);
    }

    // Visualize the goal frustum as lines.
    visualization_msgs::Marker frustum_marker;
    frustum_marker.header = header;
    frustum_marker.header.frame_id = map_frame_id_;
    frustum_marker.ns = "frustum";
    frustum_marker.id = 0;
    frustum_marker.type = visualization_msgs::Marker::LINE_LIST;
    frustum_marker.action = visualization_msgs::Marker::ADD;
    frustum_marker.pose.orientation = make_quaternion();
    frustum_marker.scale.x = 0.1f;
    frustum_marker.color = eigen_to_color(color_goal_);
    frustum_marker.color.a = 0.5f;
    frustum_marker.lifetime = ros::Duration(0.0);
    frustum_marker.frame_locked = true;
    // Add the frustum vertices to the message.
    const Eigen::Matrix4f goal_T_MC = goal_view.goalT_MB() * supereight_config_.T_BC;
    const auto& v = sensor_.frustum_vertices_;
    // Near plane.
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(0)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(1)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(1)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(2)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(2)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(3)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(3)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(0)).head<3>()));
    // Far plane.
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(4)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(5)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(5)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(6)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(6)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(7)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(7)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(4)).head<3>()));
    // Sides.
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(0)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(4)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(1)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(5)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(2)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(6)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(3)).head<3>()));
    frustum_marker.points.push_back(eigen_to_point((goal_T_MC * v.col(7)).head<3>()));
    map_goal_pub_.publish(frustum_marker);

    // Visualize the goal 360 degree raycasting rays as lines.
    visualization_msgs::Marker ray_marker;
    ray_marker.header = header;
    ray_marker.header.frame_id = map_frame_id_;
    ray_marker.ns = "rays";
    ray_marker.id = 0;
    ray_marker.type = visualization_msgs::Marker::LINE_LIST;
    ray_marker.action = visualization_msgs::Marker::ADD;
    ray_marker.pose.orientation = make_quaternion();
    ray_marker.scale.x = 0.02f;
    ray_marker.color = eigen_to_color(color_goal_);
    ray_marker.color.a = 0.25f;
    ray_marker.lifetime = ros::Duration(0.0);
    ray_marker.frame_locked = true;
    // Add the rays to the message
    const Image<Eigen::Vector3f> rays_M = goal_view.rays();
    const Eigen::Matrix4f& goal_T_MB = goal_view.goalT_MB();
    for (size_t i = 0; i < rays_M.size(); i++) {
        ray_marker.points.push_back(eigen_to_point(goal_T_MB.topRightCorner<3, 1>()));
        ray_marker.points.push_back(eigen_to_point(rays_M[i]));
    }
    map_goal_pub_.publish(ray_marker);
}



void SupereightNode::hideGoal()
{
    visualization_msgs::Marker del_marker;
    del_marker.action = visualization_msgs::Marker::DELETEALL;
    map_goal_pub_.publish(del_marker);
}



void SupereightNode::visualizeMAV()
{
    const float diameter = 2.0f * supereight_config_.robot_radius;
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = body_frame_id_;
    // Initialize the Sphere message at the center of the body frame
    visualization_msgs::Marker mav_marker;
    mav_marker = visualization_msgs::Marker();
    mav_marker.header = header;
    mav_marker.ns = "sphere";
    mav_marker.id = 1;
    mav_marker.type = visualization_msgs::Marker::SPHERE;
    mav_marker.action = visualization_msgs::Marker::ADD;
    mav_marker.pose.position = make_point();
    mav_marker.pose.orientation = make_quaternion();
    mav_marker.scale = make_vector3(diameter);
    mav_marker.color = eigen_to_color(color_mav_sphere_);
    mav_marker.lifetime = ros::Duration(0.0);
    mav_marker.frame_locked = true;
    mav_vis_pub_.publish(mav_marker);
}



void SupereightNode::visualizePoseHistory()
{
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = world_frame_id_;
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "pose_history";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = make_point();
    marker.pose.orientation = make_quaternion();
    marker.scale = make_vector3(0.1f);
    marker.color = eigen_to_color(color_pose_history_);
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = true;
    // Add all visited points
    {
        const auto& T_WB_history = planner_->getT_WBHistory();
        marker.points.resize(T_WB_history.size());
        for (size_t i = 0; i < T_WB_history.size(); ++i) {
            const Eigen::Vector3f t_WB = T_WB_history[i].topRightCorner<3, 1>();
            marker.points[i].x = t_WB.x();
            marker.points[i].y = t_WB.y();
            marker.points[i].z = t_WB.z();
        }
    }
    pose_history_pub_.publish(marker);
}



void SupereightNode::visualizePoseGridHistory()
{
    // Initialize the message header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_id_;
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "pose_grid_history";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = make_point();
    marker.pose.orientation = make_quaternion();
    marker.scale = make_vector3(1.0f);
    marker.color = eigen_to_color(color_frontier_);
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = true;
    // Add all mesh triangles
    const TriangleMesh mesh = planner_->getPoseGridHistory().wedgeMesh();
    marker.points.resize(Triangle::num_vertexes * mesh.size());
    for (size_t t = 0; t < mesh.size(); t++) {
        for (int v = 0; v < Triangle::num_vertexes; v++) {
            marker.points[Triangle::num_vertexes * t + v].x = mesh[t].vertexes[v].x();
            marker.points[Triangle::num_vertexes * t + v].y = mesh[t].vertexes[v].y();
            marker.points[Triangle::num_vertexes * t + v].z = mesh[t].vertexes[v].z();
        }
    }
    pose_history_pub_.publish(marker);
}



void SupereightNode::visualizeEnvironmentAABB()
{
    const auto& edges = pipeline_->environmentAABBEdgesM();
    visualization_msgs::Marker aabb_marker;
    aabb_marker.header.stamp = ros::Time::now();
    aabb_marker.header.frame_id = map_frame_id_;
    aabb_marker.ns = "environment_aabb";
    aabb_marker.id = 0;
    aabb_marker.type = visualization_msgs::Marker::LINE_LIST;
    aabb_marker.action = visualization_msgs::Marker::ADD;
    aabb_marker.pose.position = make_point();
    aabb_marker.pose.orientation = make_quaternion();
    aabb_marker.scale.x = 0.05;
    aabb_marker.color = eigen_to_color(color_pose_history_);
    aabb_marker.lifetime = ros::Duration(0.0);
    aabb_marker.frame_locked = true;
    aabb_marker.points.reserve(edges.size());
    for (const auto& vertex : edges) {
        aabb_marker.points.emplace_back(eigen_to_point(vertex));
    }
    limit_pub_.publish(aabb_marker);
}



void SupereightNode::visualizeSamplingAABB()
{
    const auto& edges = planner_->samplingAABBEdgesM();
    visualization_msgs::Marker aabb_marker;
    aabb_marker.header.stamp = ros::Time::now();
    aabb_marker.header.frame_id = map_frame_id_;
    aabb_marker.ns = "sampling_aabb";
    aabb_marker.id = 0;
    aabb_marker.type = visualization_msgs::Marker::LINE_LIST;
    aabb_marker.action = visualization_msgs::Marker::ADD;
    aabb_marker.pose.position = make_point();
    aabb_marker.pose.orientation = make_quaternion();
    aabb_marker.scale.x = 0.05;
    aabb_marker.color = eigen_to_color(color_sampling_aabb_);
    aabb_marker.lifetime = ros::Duration(0.0);
    aabb_marker.frame_locked = true;
    aabb_marker.points.reserve(edges.size());
    for (const auto& vertex : edges) {
        aabb_marker.points.emplace_back(eigen_to_point(vertex));
    }
    limit_pub_.publish(aabb_marker);
}

} // namespace se
