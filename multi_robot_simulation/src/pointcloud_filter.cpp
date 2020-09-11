/*
 * Copyright (c) 2020, Vision for Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * pointcloud_filter.cpp
 * @author Luca Bartolomei, V4RL
 * @date   22.08.2019
 */

#include "multi_robot_simulation/pointcloud_filter.h"

#include <tf_conversions/tf_eigen.h>

namespace pointcloud_filter {

PointcloudFilter::PointcloudFilter(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const int agent_id)
    : nh_(nh),
      nh_private_(nh_private),
      num_agents_(1),
      agent_id_(agent_id),
      world_frame_("world"),
      agent_frame_("agent"),
      ball_radius_(2.0),
      pointcloud_topic_("/dense_stereo/pointcloud") {
  // Get parameters
  std::string ns("pointcloud_filter_node_" + std::to_string(agent_id_) + "/");
  if (!nh_.getParam(ns + "num_agents", num_agents_)) {
    ROS_WARN("[PCL Filter] Number of agents not specified");
  }

  if (!nh_.getParam(ns + "world_frame", world_frame_)) {
    ROS_WARN("[PCL Filter] World frame name not specified");
  }

  if (!nh_.getParam(ns + "agent_frame", agent_frame_)) {
    ROS_WARN("[PCL Filter] Agent frame name not specified");
  }
  agent_frame_ += "_";

  if (!nh_.getParam(ns + "ball_radius", ball_radius_)) {
    ROS_WARN("[PCL Filter] Ball radius not specified");
  }

  if (!nh_.getParam(ns + "pointcloud_topic", pointcloud_topic_)) {
    ROS_WARN("[PCL Filter] Pointcloud name not specified");
  }

  // Initialize containers for the positions
  agents_positions_.resize(num_agents_, Eigen::Vector3d::Zero());
  odometry_subs_.reserve(num_agents_);
  transf_odom_map_subs_.reserve(num_agents_);
  transf_map_world_subs_.reserve(num_agents_);

  // Transformations
  T_O_A_.resize(num_agents_, Eigen::Matrix4d::Identity());
  T_W_M_.resize(num_agents_, Eigen::Matrix4d::Identity());
  T_M_O_.resize(num_agents_, Eigen::Matrix4d::Identity());
  transformations_initialized_O_A_.resize(num_agents_, false);
  transformations_initialized_M_O_.resize(num_agents_, false);
  transformations_initialized_W_M_.resize(num_agents_, false);

  // set pcl subscriber
  pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      pointcloud_topic_, 1, &PointcloudFilter::pointCloudCallback, this);

  for (uint64_t id = 0; id < num_agents_; ++id) {
    ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>(
        "odometry_" + std::to_string(id), 10,
        boost::bind(&PointcloudFilter::odometryCallback, this, _1, id));
    odometry_subs_.push_back(odom_sub);

    ros::Subscriber transf_map_odom =
        nh_.subscribe<geometry_msgs::TransformStamped>(
            "odom_to_map" + std::to_string(id), 10,
            boost::bind(&PointcloudFilter::odomToMapCallback, this, _1, id));
    transf_odom_map_subs_.push_back(transf_map_odom);

    ros::Subscriber transf_world_map =
        nh_.subscribe<geometry_msgs::TransformStamped>(
            "map_to_world" + std::to_string(id), 10,
            boost::bind(&PointcloudFilter::mapToWorldCallback, this, _1, id));
    transf_map_world_subs_.push_back(transf_world_map);
  }

  // set pcl publisher
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "filtered_pointcloud_" + std::to_string(agent_id_), 1);

  ROS_INFO_STREAM("[PCL Filter] Initialized pcl filter for agent "
                  << agent_id_);
}

PointcloudFilter::~PointcloudFilter() {}

void PointcloudFilter::pointCloudCallback(
    const PointcloudROS::ConstPtr &pcl_msg) {
  // Get the original pointcloud in the camera frame
  PointcloudPCL::Ptr original_cloud(new PointcloudPCL);
  pcl::fromROSMsg(*pcl_msg, *original_cloud);

  // Transform the pointcloud in the world frame
  ros::Time time_to_lookup = (ros::Time)pcl_msg->header.stamp;

  if (!tf_listener_.canTransform(world_frame_, pcl_msg->header.frame_id,
                                 time_to_lookup)) {
    time_to_lookup = ros::Time(0);
    ROS_WARN(
        "[PCL Filter] Using latest TF transform instead of timestamp "
        "match.");
  }

  tf::StampedTransform tf_transform_stamped;
  try {
    tf_listener_.lookupTransform(world_frame_, pcl_msg->header.frame_id,
                                 time_to_lookup, tf_transform_stamped);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM(
        "[PCL Filter] Error getting TF transform from sensor "
        "data: "
        << ex.what());

    // In this case, since we do not have a tf, just pass the pointcloud
    // without changing it
    pcl_pub_.publish(pcl_msg);
    return;
  }

  Eigen::Affine3d affine_tranf;
  tf::transformTFToEigen(tf::Transform(tf_transform_stamped.getRotation(),
                                       tf_transform_stamped.getOrigin()),
                         affine_tranf);

  PointcloudPCL::Ptr world_cloud(new PointcloudPCL);
  pcl::transformPointCloud(*original_cloud, *world_cloud,
                           affine_tranf.cast<float>());

  // Get the positions of the other agents
  std::vector<Eigen::Vector3d> agents_positions;
  agents_positions.reserve(num_agents_ - 1);
  for (int id = 0; id < num_agents_; ++id) {
    if (id == agent_id_) {
      continue;
    }
    Eigen::Vector3d position;
    if (!getPositionAgentId(id, position)) {
      ROS_ERROR_STREAM("[PCL Filter] Position of agent "
                       << id << " could not be retreived");
      return;
    }
    agents_positions.push_back(position);
  }

  // Filter the pointcloud
  PointcloudPCL::Ptr filtered_cloud(new PointcloudPCL);

  for (PointcloudPCL::const_iterator it = world_cloud->begin();
       it != world_cloud->end(); ++it) {
    Eigen::Vector3d point(it->x, it->y, it->z);
    bool valid_point = true;
    for (int i = 0; i < agents_positions.size(); ++i) {
      valid_point &= (point - agents_positions[i]).norm() > ball_radius_;
    }
    if (valid_point) {
      filtered_cloud->points.push_back(*it);
    }
  }

  // Trasform the pointcloud from world to camera frame
  tf::transformTFToEigen(
      tf::Transform(tf_transform_stamped.inverse().getRotation(),
                    tf_transform_stamped.inverse().getOrigin()),
      affine_tranf);
  PointcloudPCL::Ptr cam_filtered_cloud(new PointcloudPCL);
  pcl::transformPointCloud(*filtered_cloud, *cam_filtered_cloud,
                           affine_tranf.cast<float>());

  // Publish
  PointcloudROS filtered_cloud_msg;
  pcl::toROSMsg(*cam_filtered_cloud, filtered_cloud_msg);
  filtered_cloud_msg.header = pcl_msg->header;

  pcl_pub_.publish(filtered_cloud_msg);
}

void PointcloudFilter::odometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg, const uint64_t agent_id) {
  // Extract the info in the message
  Eigen::Vector3d position(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
  Eigen::Quaterniond orientation;
  tf::quaternionMsgToEigen(odom_msg->pose.pose.orientation, orientation);

  // Store in the rigth transformation storage
  Eigen::Matrix4d T_O_A_id;
  T_O_A_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientation);
  T_O_A_id.block<3, 1>(0, 3) = position;
  T_O_A_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  T_O_A_[agent_id] = T_O_A_id;

  // Update flag
  transformations_initialized_O_A_[agent_id] = true;
  ROS_INFO_STREAM_ONCE(
      "[PCL Filter] Got first odometry message for "
      "agent "
      << agent_id);
}

void PointcloudFilter::odomToMapCallback(
    const geometry_msgs::TransformStampedConstPtr &transf_odom_map_msg,
    const uint64_t agent_id) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientat;
  tf::vectorMsgToEigen(transf_odom_map_msg->transform.translation, position);
  tf::quaternionMsgToEigen(transf_odom_map_msg->transform.rotation, orientat);

  Eigen::Matrix4d T_M_O_id;
  T_M_O_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientat);
  T_M_O_id.block<3, 1>(0, 3) = position;
  T_M_O_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  T_M_O_[agent_id] = T_M_O_id;

  // Update flag
  transformations_initialized_M_O_[agent_id] = true;
  ROS_INFO_STREAM_ONCE(
      "[PCL Filter] Got first odom to map "
      "transformation for agent "
      << agent_id);
}

void PointcloudFilter::mapToWorldCallback(
    const geometry_msgs::TransformStampedConstPtr &transf_map_world_msg,
    const uint64_t agent_id) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientat;
  tf::vectorMsgToEigen(transf_map_world_msg->transform.translation, position);
  tf::quaternionMsgToEigen(transf_map_world_msg->transform.rotation, orientat);

  Eigen::Matrix4d T_W_M_id;
  T_W_M_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientat);
  T_W_M_id.block<3, 1>(0, 3) = position;
  T_W_M_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  T_W_M_[agent_id] = T_W_M_id;

  // Update flag
  transformations_initialized_W_M_[agent_id] = true;
  ROS_INFO_STREAM_ONCE(
      "[PCL Filter] Got first map to odom "
      "transformation for agent "
      << agent_id);
}

bool PointcloudFilter::getPositionAgentId(const int agent_id,
                                          Eigen::Vector3d &position) const {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(world_frame_,
                                 agent_frame_ + std::to_string(agent_id),
                                 ros::Time(0), transform);
    tf::vectorTFToEigen(transform.getOrigin(), position);
    return true;
  } catch (tf::TransformException ex) {
    if (!transformations_initialized_W_M_[agent_id] ||
        !transformations_initialized_M_O_[agent_id] ||
        !transformations_initialized_O_A_[agent_id]) {
      ROS_ERROR("[PCL Filter] %s", ex.what());
      return false;
    } else {
      ROS_WARN_STREAM_THROTTLE(5, "[PCL Filter] For agent "
                                      << agent_id
                                      << " using transformations in buffer");
    }
  }

  // In this case, we use the transformations we have in the buffer
  Eigen::Matrix4d T_W_O(T_W_M_[agent_id] * T_M_O_[agent_id] * T_O_A_[agent_id]);
  position = T_W_O.block<3, 1>(0, 3);
  return true;
}

}  // end namespace pointcloud_filter
