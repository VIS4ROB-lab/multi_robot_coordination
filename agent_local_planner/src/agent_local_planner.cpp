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
 * agent_local_planner.cpp
 * @author Luca Bartolomei, V4RL
 * @date   02.08.2019
 */

#include "agent_local_planner/agent_local_planner.h"

#include <signal.h>
#include <chrono>

#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/default_topics.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

namespace mrp {

AgentLocalPlanner::AgentLocalPlanner(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      planning_spinner_(1, &planning_queue_),
      command_publishing_spinner_(1, &command_publishing_queue_),
      map_initialized_(false),
      got_global_path_(false),
      stopped_(false),
      inserted_goal_(false),
      back_to_position_hold_called_(false),
      has_odom_to_world_(false),
      path_index_(0),
      T_w_o_(Eigen::Matrix4d::Identity()),
      T_odrifted_o_(Eigen::Matrix4d::Identity()) {
  // First, get the id of the agent
  CHECK(nh_private_.getParam("agent_id", agent_id_))
      << "Agent ID not specified";
  CHECK(nh_private_.getParam("map_resolution", map_resolution_))
      << "Map resolution not specified";
  CHECK(nh_private_.getParam("odometry_frame", odometry_frame_))
      << "Odometry frame not specified";
  CHECK(nh_private_.getParam("world_frame", world_frame_))
      << "World frame not specified";

  if (!nh_private_.getParam("vel_max", vel_max_)) {
    ROS_WARN_STREAM("[Local Planner] Maximum velocity for agent "
                    << agent_id_ << " not specified. Using 1.0 m/s");
    vel_max_ = 1.0;
  }

  if (!nh_private_.getParam("vel_max_yaw", vel_max_yaw_)) {
    ROS_WARN_STREAM("[Local Planner] Maximum yaw velocity for agent "
                    << agent_id_ << " not specified. Using 1.0 rad/s");
    vel_max_yaw_ = 1.0;
  }

  if (!nh_private_.getParam("acc_max", acc_max_)) {
    ROS_WARN_STREAM("[Local Planner] Maximum acceleration for agent "
                    << agent_id_ << " not specified. Using 1.0 m/s2");
    acc_max_ = 1.0;
  }

  if (!nh_private_.getParam("acc_max_yaw", acc_max_yaw_)) {
    ROS_WARN_STREAM("[Local Planner] Maximum yaw acceleration for agent "
                    << agent_id_ << " not specified. Using 1.0 rad/s2");
    acc_max_yaw_ = 1.0;
  }

  if (!nh_private_.getParam("dt", dt_)) {
    ROS_WARN_STREAM("[Local Planner] Delta t for optimizer for agent "
                    << agent_id_ << " not specified. Using 0.5 s");
    dt_ = 0.5;
  }

  if (!nh_private_.getParam("num_opt_points", num_opt_points_)) {
    ROS_WARN_STREAM(
        "[Local Planner] Number of control points for spline "
        "for agent "
        << agent_id_ << " not specified. Using 7");
    num_opt_points_ = 7;
  }

  if (!nh_private_.getParam("distance_threshold", distance_threshold_)) {
    ROS_WARN_STREAM(
        "[Local Planner] Distance treshold for spline optimizer "
        "for agent "
        << agent_id_ << " not specified. Using 0.5 m");
    distance_threshold_ = 0.5;
  }

  if (!nh_private_.getParam("sampling_dt", sampling_dt_)) {
    ROS_WARN_STREAM("[Local Planner] Sampling Dt for interpolator for agent "
                    << agent_id_ << " not specified. Using 0.05 s");
    sampling_dt_ = 0.05;
  }

  if (!nh_private_.getParam("prediction_horizon_mpc",
                            prediction_horizon_mpc_)) {
    ROS_WARN_STREAM("[Local Planner] Prediction horizon for MPC for agent "
                    << agent_id_ << " not specified. Using 300 ns");
    prediction_horizon_mpc_ = 300;
  }

  if (!nh_private_.getParam("dt_commands", dt_commands_)) {
    ROS_WARN_STREAM("[Local Planner] Delta t for commands for agent "
                    << agent_id_ << " not specified. Using 0.5 s");
    dt_commands_ = 0.5;
  }

  if (!nh_private_.getParam("local_goal_distance", local_goal_distance_)) {
    ROS_WARN_STREAM("[Local Planner] Local goal distance for agent "
                    << agent_id_ << " not specified. Using 2.50 m");
    local_goal_distance_ = 2.50;
  }

  if (!nh_private_.getParam("scale_factor_visualization",
                            scale_factor_visualization_)) {
    ROS_WARN_STREAM("[Local Planner] Scale factor for visualization for agent "
                    << agent_id_ << " not specified. Using 1.0");
    scale_factor_visualization_ = 1.0;
  }

  if (!nh_private_.getParam("min_z", min_z_)) {
    ROS_WARN_STREAM("[Local Planner] Min z for visualization for agent "
                    << agent_id_ << " not specified. Using 0.0");
    min_z_ = 0.0;
  }

  if (!nh_private_.getParam("max_z", max_z_)) {
    ROS_WARN_STREAM("[Local Planner] Max z for visualization for agent "
                    << agent_id_ << " not specified. Using 7.0");
    max_z_ = 7.0;
  }

  // Set up log file
  log_file_.open(ros::package::getPath("agent_local_planner") +
                 "/log/local_path_" + std::to_string(agent_id_) + ".csv");
  log_file_ << "x [m], y [m], z [m]" << std::endl;

  // Adjust the odometry frame
  odometry_frame_ += "_" + std::to_string(agent_id_);

  // Initialize ROS
  initROS();

  // EWOK
  edrb_.reset(new ewok::EuclideanDistanceRingBuffer<POW>(map_resolution_, 1.0));
  b_spline_.reset(new ewok::UniformBSpline3D<POW, double>(dt_));

  // Interpolator
  poly_interpolator_ = std::unique_ptr<mrp::PolynomialInterpolator>(
      new mrp::PolynomialInterpolator(vel_max_, acc_max_, vel_max_yaw_,
                                      acc_max_yaw_, sampling_dt_));

  // Initialization completed
  ROS_INFO_STREAM("[Local Planner] Initialization for agent "
                  << agent_id_ << " is complete");
}

AgentLocalPlanner::~AgentLocalPlanner() {
  // Stop timers
  planning_timer_.stop();
  command_publishing_timer_.stop();

  // Stop spinners
  planning_spinner_.stop();
  command_publishing_spinner_.stop();

  // Close log file as well
  log_file_.close();
}

void AgentLocalPlanner::initROS() {
  // Initialize subscribers
  odometry_sub_ = nh_.subscribe(
      "odometry", 100, &mrp::AgentLocalPlanner::odometryCallback, this);

  global_path_sub_ =
      nh_.subscribe("global_path_" + std::to_string(agent_id_), 100,
                    &mrp::AgentLocalPlanner::globalPathCallback, this);

  pcl_sub_ = nh_.subscribe("filtered_pointcloud", 10,
                           &mrp::AgentLocalPlanner::pclCallback, this);

  // Initialize publishers
  occupied_marker_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "ring_buffer/occupied", 5);
  free_marker_pub_ =
      nh_private_.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
  distance_marker_pub_ = nh_private_.advertise<visualization_msgs::Marker>(
      "ring_buffer/distance", 5);
  optimal_trajectory_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "optimal_trajectory", 1);
  global_trajectory_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("global_reference",
                                                             1);
  trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  command_visual_pub_ = nh_.advertise<nav_msgs::Path>("command_visual", 10);

  // Initialize service servers
  std::string stop_srv_name = "/stop_agent_" + std::to_string(agent_id_);
  stop_srv_ = nh_.advertiseService(
      stop_srv_name, &AgentLocalPlanner::stopServiceCallback, this);
}

void AgentLocalPlanner::odometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  ROS_INFO_STREAM_ONCE("[Local Planner] Agent "
                       << agent_id_ << " received first odometry message");
  mav_msgs::eigenOdometryFromMsg(*odom_msg, &odometry_);

  if (b_spline_->size() == 0) {
    for (int i = 0; i < POW; i++) {
      b_spline_->push_back(odometry_.position_W);
    }
    init_time_ = ros::Time::now();
  }

  // if we don't have a world transformation already
  if (!has_odom_to_world_) {
    tf::StampedTransform transform_stamped;
    try {
      tf_listener_.lookupTransform(world_frame_, odometry_frame_, ros::Time(0),
                                   transform_stamped);
      Eigen::Quaterniond quaternion_eigen;
      tf::quaternionTFToEigen(transform_stamped.getRotation(),
                              quaternion_eigen);

      T_w_o_.block(0, 3, 3, 1) << transform_stamped.getOrigin().getX(),
          transform_stamped.getOrigin().getY(),
          transform_stamped.getOrigin().getZ();
      T_w_o_.block(0, 0, 3, 3)
          << quaternion_eigen.normalized().toRotationMatrix();
      T_w_o_.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;
      has_odom_to_world_ = true;
      ROS_INFO_STREAM("[Local Planner] Agent "
                      << agent_id_
                      << " has "
                         "initialized the transformation odom to world");

    } catch (tf::TransformException &ex) {
      ROS_WARN_STREAM("Error getting TF transform World - Odom: " << ex.what());
    }
  }
}

void AgentLocalPlanner::globalPathCallback(
    const nav_msgs::PathConstPtr &path_msg) {
  // Clear the command buffer as well, otherwise the robot will keep moving!
  resetLocalPlanner();

  // Check if the global path is empty - if so, then we need to stop!
  if (path_msg->poses.empty()) {
    ROS_WARN_STREAM("[Local Planner] Agent " << agent_id_
                                             << " received stop "
                                                "command");
    sendStopCommand();

    // Stop the timers
    command_publishing_timer_.stop();
    planning_timer_.stop();
    got_global_path_ = false;

    return;
  }

  ROS_INFO_STREAM("[Local Planner] Agent " << agent_id_
                                           << " received a new "
                                              "global path");

  // Check the reference frame
  if (path_msg->header.frame_id.compare(odometry_frame_) != 0) {
    ROS_ERROR_STREAM("[Local Planner] Agent " << agent_id_
                                              << " received a "
                                                 "global path in frame "
                                              << path_msg->header.frame_id
                                              << " but we expected "
                                              << odometry_frame_);
    return;
  }

  // Store the current transformation world to odom so that we can use when
  // we have to transform the commands in the drifted odometry frame. If we
  // cannot find the transformation, then use the one in memory
  tf::StampedTransform transform_stamped;
  try {
    tf_listener_.lookupTransform(world_frame_, odometry_frame_, ros::Time(0),
                                 transform_stamped);
    Eigen::Quaterniond quaternion_eigen;
    tf::quaternionTFToEigen(transform_stamped.getRotation(), quaternion_eigen);

    T_w_o_.block(0, 3, 3, 1) << transform_stamped.getOrigin().getX(),
        transform_stamped.getOrigin().getY(),
        transform_stamped.getOrigin().getZ();
    T_w_o_.block(0, 0, 3, 3)
        << quaternion_eigen.normalized().toRotationMatrix();
    T_w_o_.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;
  } catch (tf::TransformException &ex) {
    ROS_WARN_STREAM("Error getting TF transform World - Odom: " << ex.what());
  }

  // Create the polynomial optimizer
  if (poly_traj_) {
    poly_traj_.reset();
  }

  Eigen::Vector4d limits(vel_max_, acc_max_, 0.0, 0.0);
  ewok::Polynomial3DOptimization<NOPT> poly_3d_opt(limits);

  {
    std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    typename ewok::Polynomial3DOptimization<NOPT>::Vector3Array path;

    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
      Eigen::Vector3d position(path_msg->poses[i].pose.position.x,
                               path_msg->poses[i].pose.position.y,
                               path_msg->poses[i].pose.position.z);
      path.push_back(position);
    }

    poly_traj_ = poly_3d_opt.computeTrajectory(path);

    if (global_trajectory_marker_pub_.getNumSubscribers() > 0) {
      visualization_msgs::MarkerArray trajectory_marker;
      poly_traj_->getVisualizationMarkerArray(
          trajectory_marker, "global_reference", Eigen::Vector3d(1, 0, 1));
      for (size_t i = 0; i < trajectory_marker.markers.size(); ++i) {
        trajectory_marker.markers[i].header.frame_id = odometry_frame_;

        // Increase the size as well
        trajectory_marker.markers[i].scale.x *= scale_factor_visualization_;
        trajectory_marker.markers[i].scale.y *= scale_factor_visualization_;
        trajectory_marker.markers[i].scale.z *= scale_factor_visualization_;
      }

      global_trajectory_marker_pub_.publish(trajectory_marker);
    }

    // Reset the spline optimizer with the new global path
    spline_optimization_.reset(
        new ewok::UniformBSpline3DOptimization<POW>(poly_traj_, dt_));

    for (int i = 0; i < num_opt_points_; ++i) {
      spline_optimization_->addControlPoint(
          Eigen::Vector3d(path_msg->poses[0].pose.position.x,
                          path_msg->poses[0].pose.position.y,
                          path_msg->poses[0].pose.position.z));
    }

    spline_optimization_->setNumControlPointsOptimized(num_opt_points_);
    spline_optimization_->setDistanceBuffer(edrb_);
    spline_optimization_->setDistanceThreshold(distance_threshold_);
    spline_optimization_->setLimits(limits);
  }

  // If not done yet, start the spinners
  if (!got_global_path_) {
    ROS_INFO("[Local Planner] Started timers");

    // Planning timer
    ros::TimerOptions timer_options_planning(
        ros::Duration(dt_),
        boost::bind(&mrp::AgentLocalPlanner::plannerTimerCallback, this, _1),
        &planning_queue_);

    planning_timer_ = nh_.createTimer(timer_options_planning);
    planning_spinner_.start();

    // Command timer
    ros::TimerOptions timer_options_command(
        ros::Duration(dt_commands_),
        boost::bind(&mrp::AgentLocalPlanner::commandPublishTimerCallback, this,
                    _1),
        &command_publishing_queue_);

    command_publishing_timer_ = nh_.createTimer(timer_options_command);
    command_publishing_spinner_.start();

    // Call the back to position hold for mpc
    if (!back_to_position_hold_called_) {
      ros::ServiceClient back_to_position_hold =
          nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");
      if (back_to_position_hold.exists()) {
        std_srvs::Empty empty_srv;
        if (back_to_position_hold.call(empty_srv)) {
          ROS_INFO_STREAM("Called back_to_position_hold for agent "
                          << agent_id_);
        } else {
          ROS_ERROR_STREAM("Could not call back_to_position_hold for agent "
                           << agent_id_);
        }
      } else {
        ROS_WARN_STREAM("back_to_position_hold does not exist for agent "
                        << agent_id_);
      }
      back_to_position_hold_called_ = true;
    }

    got_global_path_ = true;
  }

  // Store the goal
  goal_ << path_msg->poses.back().pose.position.x,
      path_msg->poses.back().pose.position.y,
      path_msg->poses.back().pose.position.z;
  ROS_INFO_STREAM("[Local Planner] Set goal to " << goal_.transpose());
}

void AgentLocalPlanner::pclCallback(
    const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {
  // Timings
  ROS_INFO_ONCE("[Local Planner] Got first pointcloud");
  auto start_time = std::chrono::high_resolution_clock::now();

  // Transform the pointcloud in the odometry frame
  tf::StampedTransform tf_transform_stamped;
  try {
    tf_listener_.lookupTransform(odometry_frame_, pcl_msg->header.frame_id,
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_cam_frame(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_odom_frame(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*pcl_msg, *pointcloud_in_cam_frame);
  pcl::transformPointCloud(*pointcloud_in_cam_frame, *pointcloud_in_odom_frame,
                           affine_tranf.cast<float>());

  // Put the points in the pointcloud in the right format
  ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;
  for (size_t i = 0; i < pointcloud_in_odom_frame->points.size(); ++i) {
    cloud.push_back(Eigen::Vector4f(pointcloud_in_odom_frame->points[i].x,
                                    pointcloud_in_odom_frame->points[i].y,
                                    pointcloud_in_odom_frame->points[i].z,
                                    1.0));
  }

  // Get the origin for the pointcloud
  Eigen::Vector3f origin(
      static_cast<float>(tf_transform_stamped.getOrigin().getX()),
      static_cast<float>(tf_transform_stamped.getOrigin().getY()),
      static_cast<float>(tf_transform_stamped.getOrigin().getZ()));

  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  if (!map_initialized_) {
    Eigen::Vector3i idx;
    edrb_->getIdx(origin, idx);
    edrb_->setOffset(idx);
    map_initialized_ = true;
    ROS_INFO("[Local Planner] Initialized local map");
  } else {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb_->getIdx(origin, origin_idx);
    offset = edrb_->getVolumeCenter();
    diff = origin_idx - offset;
    while (diff.array().any()) {
      edrb_->moveVolume(diff);
      offset = edrb_->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }

  // Insert the pointcloud in the buffer
  edrb_->insertPointCloud(cloud, origin);
  // Publish the visualization
  if (occupied_marker_pub_.getNumSubscribers() > 0 ||
      free_marker_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker marker_occupied, marker_free;
    edrb_->getMarkerOccupied(marker_occupied);
    edrb_->getMarkerFree(marker_free);

    // Overwrite reference frame name
    marker_occupied.header.frame_id = odometry_frame_;
    marker_free.header.frame_id = odometry_frame_;

    // Recolor the occupancy map
    for (size_t i = 0; i < marker_occupied.points.size(); ++i) {
      double z = marker_occupied.points[i].z;
      marker_occupied.colors.push_back(
          percentToColor(colorizeMapByHeight(z, min_z_, max_z_)));
    }

    occupied_marker_pub_.publish(marker_occupied);
    free_marker_pub_.publish(marker_free);
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  ROS_INFO_STREAM_THROTTLE(
      1, "[Local Planner] Inserted pointcloud in "
             << std::chrono::duration_cast<std::chrono::nanoseconds>(end_time -
                                                                     start_time)
                        .count() *
                    1e-9
             << " s");
}

bool AgentLocalPlanner::stopServiceCallback(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res) {
  // Stop the whole planner logic
  if (stopped_) {
    return true;
  }

  ROS_WARN_STREAM("[Local Planner] Agent " << agent_id_
                                           << " received whole "
                                              "stop command");
  resetLocalPlanner();
  sendStopCommand();

  command_publishing_timer_.stop();
  planning_timer_.stop();
  got_global_path_ = false;
  stopped_ = true;

  return true;
}

void AgentLocalPlanner::plannerTimerCallback(const ros::TimerEvent &e) {
  // Timings
  auto start_time = std::chrono::high_resolution_clock::now();

  // Update map and get the point to send to controller
  Eigen::Vector3d pc;
  {
    std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    edrb_->updateDistance();
    spline_optimization_->optimize();
    pc = spline_optimization_->getFirstOptimizationPoint();
  }

  // Check if we have reached the current goal - if so, there is no need to
  // continue replanning
  if ((goal_ - odometry_.position_W).norm() <= map_resolution_) {
    ROS_INFO_STREAM_THROTTLE(10, "[Local Planner] Agent "
                                     << agent_id_
                                     << " has reached current goal");
    // Force reset to avoid doing weird stuff
    resetLocalPlanner();
    sendStopCommand();
    return;
  }

  if (inserted_goal_) {
    return;
  }

  // Using the odometry can cause a deadlock where the agent does not move
  // anymore, because the last point in the spline is too far away. This is
  // ok for some time, but in some situations the buffers gets empty and then
  // the agent is stuck:
  //   if((pc - odometry_.position_W).norm() > local_goal_distance_) { return; }
  // Instead, use the spline information
  {
    std::lock_guard<std::recursive_mutex> lock(commands_mutex_);
    if (interpolated_path_queue_.size() > 0 &&
        (pc - interpolated_path_queue_[path_index_].head(3)).norm() >
            local_goal_distance_) {
      return;
    }
  }

  // Publish visualization stuff
  if (optimal_trajectory_pub_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray trajectory_marker;
    {
      std::lock_guard<std::recursive_mutex> lock(map_mutex_);
      spline_optimization_->getMarkers(trajectory_marker);
    }

    // Transform to the drifted odometry frame
    updateTransformToDriftedOdom();

    for (size_t i = 0; i < trajectory_marker.markers.size(); ++i) {
      trajectory_marker.markers[i].header.frame_id = odometry_frame_;

      for (size_t j = 0; j < trajectory_marker.markers[i].points.size(); ++j) {
        Eigen::Vector3d posit;
        tf::pointMsgToEigen(trajectory_marker.markers[i].points[j], posit);

        Eigen::Vector4d posit_transf(
            T_odrifted_o_ * Eigen::Vector4d(posit(0), posit(1), posit(2), 1.0));
        trajectory_marker.markers[i].points[j].x = posit_transf(0);
        trajectory_marker.markers[i].points[j].y = posit_transf(1);
        trajectory_marker.markers[i].points[j].z = posit_transf(2);

        // Increase the size as well
        trajectory_marker.markers[i].scale.x *= scale_factor_visualization_;
        trajectory_marker.markers[i].scale.y *= scale_factor_visualization_;
        trajectory_marker.markers[i].scale.z *= scale_factor_visualization_;
      }
    }
    optimal_trajectory_pub_.publish(trajectory_marker);
  }

  // Add the control point to the b-spline interpolator
  {
    std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    if (b_spline_->size() != 0) {
      b_spline_->push_back(pc);
    } else {
      // If we don't have anything in the b-spline, it means something is wrong
      // and that we have to wait for the odometry
      return;
    }
    spline_optimization_->addLastControlPoint();

    double local_t = (ros::Time::now() - init_time_).toSec();
    if (local_t > b_spline_->maxValidTime()) {
      b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size() - 1));
    }
  }

  /** Store this info for checks in the future: we were trying to calculate
   * the yaw for the trajectory by using the points in the spline. However,
   * some points were so close that the yaw to connect them did not make
   * sense at all and cause the drone rotate on spot whenever it received a
   * new global path or reached the goal. So now we interpolate the local
   * path directly where we are sure that the control points make sense.
  // Get the orientation from the spline as well
  mav_msgs::EigenTrajectoryPoint command_trajectory;
  getTrajectoryPoint(local_t, command_trajectory);
  double yaw = command_trajectory.getYaw();

  // Check if the current yaw is fine or if it is just an artifact due to the
  // buffering strategy
  if(yaw == 0.0) {
    yaw = yaw_old_;
  }
  yaw_old_ = yaw;

  NOTE: If you want to use this old piece of code, then in the lock with the
  mutex do not use the 'pc' variable, but use the info stored in the
  'command_trajectory' var
  */

  // Store the control pose to the storage
  {
    std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
    if (path_queue_.empty() ||
        (pc - path_queue_.back().head(3)).norm() > distance_threshold_) {
      Eigen::VectorXd point(5);
      // here time and yaw does not matter apparently
      point << pc.x(), pc.y(), pc.z(), 0.0, 0.0;
      path_queue_.push_back(point);

      // Store to log file
      if (log_file_.is_open()) {
        log_file_ << pc.x() << "," << pc.y() << "," << pc.z() << std::endl;
      }

    } else if ((pc - goal_).norm() < local_goal_distance_) {
      Eigen::VectorXd point(5);
      point << goal_.x(), goal_.y(), goal_.z(), 0.0, 0.0;
      path_queue_.push_back(point);
      inserted_goal_ = true;

      // Store to log file
      if (log_file_.is_open()) {
        log_file_ << goal_.x() << "," << goal_.y() << "," << goal_.z()
                  << std::endl;
      }
    }

    // Interpolate the local path: it makes sense only if you have at least 2
    // points (otherwise the yaw is not defined) or if we have inserted the
    // goal (in this case we will have 2 positions, ie the current odometry
    // and the goal itself)
    if (path_queue_.size() > 1 || inserted_goal_) {
      // Extract the yaw from the positions in the local path
      getYawsAlongLocalPath();
      poly_interpolator_->interpolate(path_queue_, &interpolated_path_queue_,
                                      odometry_.velocity_B);
    }
  }

  // Publish the distance markers as well
  if (distance_marker_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker marker_dist;
    edrb_->getMarkerDistance(marker_dist, distance_threshold_);

    marker_dist.header.frame_id = odometry_frame_;
    distance_marker_pub_.publish(marker_dist);
  }

  // Print timings
  auto end_time = std::chrono::high_resolution_clock::now();
  ROS_INFO_STREAM_THROTTLE(
      1, "[Local Planner] Computed path in "
             << std::chrono::duration_cast<std::chrono::nanoseconds>(end_time -
                                                                     start_time)
                        .count() *
                    1e-9
             << " s");
}

void AgentLocalPlanner::commandPublishTimerCallback(
    const ros::TimerEvent &event) {
  // Here we assume that all the points in the path_queue_ storage have a
  // temporal distance of dt_ and that the initial time of the buffer is 0
  ROS_INFO_ONCE("[Local Planner] Started timer for command callback");
  std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
  if (path_queue_.empty() || trajectory_pub_.getNumSubscribers() == 0) {
    ROS_INFO_THROTTLE(10, "[Local Planner] Not sending commands...");
    return;
  }

  // Check how many poses in the buffer we have to publish to the controller
  constexpr size_t kQueueBuffer = 0;
  size_t number_to_publish =
      std::min<size_t>(std::floor(dt_commands_ / sampling_dt_),
                       interpolated_path_queue_.size() - path_index_);

  size_t starting_index = 0;
  if (path_index_ != 0) {
    starting_index = path_index_ + kQueueBuffer;
    if (starting_index >= interpolated_path_queue_.size()) {
      starting_index = path_index_;
    }
  }

  int mpc_prediction_horizon =
      static_cast<int>(std::floor(prediction_horizon_mpc_ / sampling_dt_));
  size_t number_to_publish_with_buffer = std::min<size_t>(
      number_to_publish + mpc_prediction_horizon - kQueueBuffer,
      interpolated_path_queue_.size() - starting_index);

  // Print statistics for debug
  if (DEBUG) {
    std::cout << "======================================" << std::endl;
    std::cout << "Number to publish            : " << number_to_publish
              << std::endl;
    std::cout << "Number to publish with buffer: "
              << number_to_publish_with_buffer << std::endl;
    std::cout << "Start index                  : " << starting_index
              << std::endl;
    std::cout << "Interpolated Queue size      : "
              << interpolated_path_queue_.size() << std::endl;
    std::cout << "Current path index           : " << path_index_ << std::endl;
  }

  // Get the transformation from odometry to drifted odometry
  updateTransformToDriftedOdom();

  // Final limits of the buffer to be published
  auto first_sample = interpolated_path_queue_.begin() + starting_index;
  auto last_sample = first_sample + number_to_publish_with_buffer;

  trajectory_msgs::MultiDOFJointTrajectory command_trajectory;
  command_trajectory.header.frame_id = odometry_frame_;
  command_trajectory.header.stamp = ros::Time::now();
  command_trajectory.header.seq = 0;

  nav_msgs::Path command_path;  // For visualization purposes
  command_path.header = command_trajectory.header;

  for (std::vector<Eigen::VectorXd>::iterator it = first_sample;
       it < last_sample; ++it) {
    // Pose message
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
    geometry_msgs::PoseStamped pose_stamped;

    // Express all the points in the drifted odometry frame (position and yaw)
    Eigen::Vector3d drifted_pos(
        (T_odrifted_o_ * Eigen::Vector4d((*it).x(), (*it).y(), (*it).z(), 1.0))
            .head(3));

    Eigen::Quaterniond quat_eigen;
    tf::quaternionTFToEigen(tf::createQuaternionFromYaw((*it)(3)), quat_eigen);
    Eigen::Matrix3d R_w_o(quat_eigen);
    Eigen::Matrix3d R_w_od(R_w_o * T_odrifted_o_.block(0, 0, 3, 3).inverse());
    tf::Quaternion quat_tf_od;
    tf::quaternionEigenToTF(Eigen::Quaterniond(R_w_od), quat_tf_od);
    double drifted_yaw(tf::getYaw(quat_tf_od));

    // Store the information
    trajectory_point.transforms.resize(1);
    tf::vectorEigenToMsg(drifted_pos,
                         trajectory_point.transforms[0].translation);
    trajectory_point.transforms[0].rotation =
        tf::createQuaternionMsgFromYaw(drifted_yaw);

    pose_stamped.header.frame_id = odometry_frame_;
    tf::pointEigenToMsg(drifted_pos, pose_stamped.pose.position);
    pose_stamped.pose.orientation = trajectory_point.transforms[0].rotation;

    try {
      trajectory_point.time_from_start =
          ros::Duration(((*it)(4) - (*first_sample)(4)) * 1e-9);
      pose_stamped.header.stamp =
          ros::Time(((*it)(4) - (*first_sample)(4)) * 1e-9);
    } catch (std::runtime_error &ex) {
      ROS_ERROR("[Local Planner] Exception: [%s]", ex.what());
      ROS_INFO_STREAM("Start time: " << (*first_sample)(4) * 1e-9
                                     << " s - End time: " << (*it)(4) * 1e-9
                                     << " s");
      return;
    }
    command_trajectory.points.push_back(trajectory_point);
    command_path.poses.push_back(pose_stamped);
  }

  // Publish
  ROS_INFO_ONCE("[Local Planner] Published first trajectory to controller");
  trajectory_pub_.publish(command_trajectory);
  command_visual_pub_.publish(command_path);

  // Update the starting index to publish commands
  path_index_ += number_to_publish;
}

void AgentLocalPlanner::sendStopCommand() const {
  // Publish stop command to the MPC
  trajectory_msgs::MultiDOFJointTrajectory command_trajectory;
  command_trajectory.header.frame_id = odometry_frame_;
  command_trajectory.header.stamp = ros::Time::now();
  command_trajectory.header.seq = 0;

  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms.resize(1);
  tf::vectorEigenToMsg(odometry_.position_W, point.transforms[0].translation);
  tf::quaternionEigenToMsg(odometry_.orientation_W_B,
                           point.transforms[0].rotation);
  command_trajectory.points.push_back(point);

  ROS_INFO("[Local Planner] Sending stop command to controller");
  trajectory_pub_.publish(command_trajectory);
}

void AgentLocalPlanner::resetLocalPlanner() {
  ROS_INFO("[Local Planner] Reset planner");
  std::lock_guard<std::recursive_mutex> guard(commands_mutex_);

  // Clear containers
  path_queue_.clear();
  interpolated_path_queue_.clear();

  // Insert the current odometry in the queue
  Eigen::VectorXd odom_eigen(5);
  odom_eigen.head(3) = odometry_.position_W;
  odom_eigen(3) = odometry_.getYaw();
  odom_eigen(4) = 0.0;
  path_queue_.push_back(odom_eigen);

  // Reinitialize indices
  path_index_ = 0;
  init_time_ = ros::Time::now();
  inserted_goal_ = false;

  // B-Spline reset
  b_spline_.reset(new ewok::UniformBSpline3D<POW, double>(dt_));
}

bool AgentLocalPlanner::getTrajectoryPoint(
    double t, mav_msgs::EigenTrajectoryPoint &command_trajectory) const {
  // Check the time interval
  if (t > b_spline_->maxValidTime()) {
    t = b_spline_->maxValidTime();
  }
  if (t < b_spline_->minValidTime()) {
    t = b_spline_->minValidTime();
  }

  // Evaluate the spline at query time
  command_trajectory.position_W = b_spline_->evaluate(t, 0);
  command_trajectory.velocity_W = b_spline_->evaluate(t, 1);
  command_trajectory.acceleration_W = b_spline_->evaluate(t, 2);

  // Evaluate the delta position by looking an eps in the future
  static const double eps = sampling_dt_;
  static const double delta = 0.02;
  Eigen::Vector3d d_t =
      b_spline_->evaluate(t + eps, 0) - command_trajectory.position_W;

  if (std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta) {
    double yaw = std::atan2(d_t[1], d_t[0]);
    command_trajectory.setFromYaw(yaw);
    Eigen::Vector3d d_t_e =
        b_spline_->evaluate(t + 2.0 * eps, 0) - b_spline_->evaluate(t + eps, 0);

    if (std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta) {
      double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
      double yaw_rate = (yaw_e - yaw) / eps;
      command_trajectory.setFromYawRate(yaw_rate);
    } else {
      command_trajectory.setFromYawRate(0);
    }
    return true;
  }
  return false;
}

void AgentLocalPlanner::updateTransformToDriftedOdom() {
  // Extract the transformation from odometry to world frame (ie drifted one)
  tf::StampedTransform transform_stamped;
  try {
    tf_listener_.lookupTransform(world_frame_, odometry_frame_, ros::Time(0),
                                 transform_stamped);
  } catch (tf::TransformException &ex) {
    // In this case, keep using the one in memory
    ROS_ERROR_STREAM("Error getting TF transform World - Odom: " << ex.what());
    return;
  }

  // Store the transformation in Eigen format
  Eigen::Quaterniond quaternion_eigen;
  tf::quaternionTFToEigen(transform_stamped.getRotation(), quaternion_eigen);

  Eigen::Matrix4d T_w_odrift;
  T_w_odrift.block(0, 3, 3, 1) << transform_stamped.getOrigin().getX(),
      transform_stamped.getOrigin().getY(),
      transform_stamped.getOrigin().getZ();
  T_w_odrift.block(0, 0, 3, 3)
      << quaternion_eigen.normalized().toRotationMatrix();
  T_w_odrift.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;

  // Concatenate the two transformation to get T_odrifted_o
  T_odrifted_o_ = T_w_odrift.inverse() * T_w_o_;
}

void AgentLocalPlanner::getYawsAlongLocalPath() {
  double yaw;
  for (size_t i = 0; i < path_queue_.size() - 1; ++i) {
    Eigen::Vector3d p0(path_queue_[i].head(3));
    Eigen::Vector3d p1(path_queue_[i + 1].head(3));

    yaw = std::atan2((p1 - p0)(1), (p1 - p0)(0));
    path_queue_[i](3) = yaw;
  }

  // Make sure we use the odometry info for the first point
  path_queue_[0](3) = odometry_.getYaw();

  // Add last yaw to be equal to the previous one
  path_queue_.back()(3) = yaw;
}

inline double AgentLocalPlanner::colorizeMapByHeight(double z, double min_z,
                                                     double max_z) const {
  return (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0), 1.0));
}

std_msgs::ColorRGBA AgentLocalPlanner::percentToColor(double h) const {
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)
  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

}  // end namespace mrp
