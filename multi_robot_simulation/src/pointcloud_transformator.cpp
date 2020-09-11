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
 * @author Luca Bartolomei, V4RL
 * @date   31.07.2019
 */

#include "multi_robot_simulation/pointcloud_transformator.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <comm_msgs/pcl_transform.h>
#include <Eigen/Eigen>

namespace mrp {

PointcloudTransformator::PointcloudTransformator(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  if (readParameters()) {
    initSubscribers();
    initPublisher();
  }
}

bool PointcloudTransformator::readParameters() {
  // Read how many sources
  bool load_success = nh_private_.getParam("num_agents", num_agents_);
  CHECK(load_success) << "Failed to load the number of agents.";
  CHECK_GT(num_agents_, 0) << "We need to have at least one agent.";

  // Read the reference frames names
  load_success &= nh_private_.getParam("world_frame", world_frame_);
  CHECK(load_success) << "Failed to load the world reference frame name";

  load_success &= nh_private_.getParam("odom_frame", odom_frame_);
  CHECK(load_success) << "Failed to load the odometry reference frame name";

  load_success &= nh_private_.getParam("agent_frame", agent_frame_);
  CHECK(load_success) << "Failed to load the agent reference frame name";

  load_success &= nh_private_.getParam("publish_pcl_world", publish_pcl_world_);
  CHECK(load_success) << "Failed to load if pcl should be published in world";

  // Read and create initial reference frames
  for (size_t id = 0; id < num_agents_; ++id) {
    std::string agent_name = "agent_" + std::to_string(id);

    // Get the agent namespaces
    std::string agent_namespace = agent_name + "/namespace";
    std::string ns;
    load_success &= nh_private_.getParam(agent_namespace, ns);
    CHECK(load_success) << "Failed to read agent namespace";
    agents_ns_.push_back(ns);
  }
  return true;
}

void PointcloudTransformator::initSubscribers() {
  // Set up all callbacks for odometry info for all the agents
  pointcloud_subs_.reserve(num_agents_);

  for (uint64_t id = 0; id < num_agents_; ++id) {
    std::string pcl_ns =
        agents_ns_[id] + "/vi_sensor/camera_depth/depth/points";
    ROS_INFO_STREAM("Subscribing to: " << pcl_ns);

    ros::Subscriber tmp_sub_pcl = nh_.subscribe<sensor_msgs::PointCloud2>(
        pcl_ns, 10,
        boost::bind(&PointcloudTransformator::pointcloudCallback, this, _1,
                    id));
    pointcloud_subs_.push_back(tmp_sub_pcl);
  }
}

void PointcloudTransformator::initPublisher() {
  pcl_transform_pubs_.reserve(num_agents_);
  for (uint64_t id = 0; id < num_agents_; ++id) {
    std::string pcl_topic(agents_ns_[id] + "/transform_pointcloud");
    ros::Publisher tmp_pub_pcl =
        nh_.advertise<comm_msgs::pcl_transform>(pcl_topic, 10);
    pcl_transform_pubs_.push_back(tmp_pub_pcl);

    std::string downsampled_pcl_topic(agents_ns_[id] +
                                      "/downsampled_pointcloud");
    ros::Publisher tmp_pub_downsampled_pcl =
        nh_.advertise<sensor_msgs::PointCloud2>(downsampled_pcl_topic, 10);
    pointcloud_pubs_.push_back(tmp_pub_downsampled_pcl);
  }
}

void PointcloudTransformator::pointcloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &pcl_msg, const uint64_t agent_id) {
  // Transform the pointcloud and publish it as a ROS message
  PointcloudPCL::Ptr pointcloud_in_cam_frame(new PointcloudPCL);
  PointcloudPCL::Ptr pointcloud_in_cam_frame_filtered(new PointcloudPCL);
  PointcloudROS filtered_cloud_msg;

  pcl::fromROSMsg(*pcl_msg, *pointcloud_in_cam_frame);

  // Filter pointcloud
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
  filter.setInputCloud(pointcloud_in_cam_frame);
  filter.setLeafSize(0.15f, 0.15f, 0.15f);
  filter.filter(*pointcloud_in_cam_frame_filtered);

  pcl::toROSMsg(*pointcloud_in_cam_frame_filtered, filtered_cloud_msg);
  filtered_cloud_msg.header.stamp = ros::Time::now();
  filtered_cloud_msg.header.frame_id = pcl_msg->header.frame_id;

  if (publish_pcl_world_) {
    // Extract the transformation from the camera frame to the world frame
    tf::StampedTransform tf_transform_stamped;
    try {
      tf_listener_.lookupTransform(world_frame_, pcl_msg->header.frame_id,
                                   ros::Time(0), tf_transform_stamped);
    } catch (tf::TransformException &ex) {
      ROS_ERROR_STREAM(
          "Error getting TF transform from sensor data: " << ex.what());
      return;
    }

    Eigen::Affine3d affine_tranf;
    tf::transformTFToEigen(tf::Transform(tf_transform_stamped.getRotation(),
                                         tf_transform_stamped.getOrigin()),
                           affine_tranf);
    comm_msgs::pcl_transform pcl_transform_msg;
    pcl_transform_msg.header.frame_id = world_frame_;
    pcl_transform_msg.header.seq = 0;
    pcl_transform_msg.header.stamp = ros::Time::now();

    geometry_msgs::TransformStamped world_transform;
    world_transform.header.frame_id = world_frame_;
    world_transform.header.seq = 0;
    world_transform.header.stamp = ros::Time::now();
    world_transform.child_frame_id = pcl_msg->header.frame_id;
    tf::transformTFToMsg(tf_transform_stamped, world_transform.transform);

    pcl_transform_msg.fusedPointcloud = filtered_cloud_msg;
    pcl_transform_msg.worldTransform = world_transform;

    ROS_INFO_ONCE("Published first transformed pointcloud");
    pcl_transform_pubs_[agent_id].publish(pcl_transform_msg);

    // Publish debug pointcloud
    if (pointcloud_pubs_[agent_id].getNumSubscribers() > 0) {
      ROS_INFO_ONCE("Published first filtered pointcloud");
      pointcloud_pubs_[agent_id].publish(filtered_cloud_msg);
    }
  }
}

}  // end namespace mrp
