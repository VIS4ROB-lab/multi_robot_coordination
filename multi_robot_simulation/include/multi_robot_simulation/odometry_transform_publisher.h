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
 * @brief  Main class for simulating the drift of the odometry reference
 *         frame and publish the transformation between the world and the
 *         simulated odometry frames for multiple agents
 * @date   31.07.2019
 */

#pragma once

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace mrp {

struct RandomWalkParams {
  double mean_pos;
  double cov_pos;

  double mean_ang;
  double cov_ang;
};

class OdometryTransformPublisher {
 public:
  /**
   * @brief Constructor of the class
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
  OdometryTransformPublisher(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private);

  /**
   * @brief Destructor
   */
  ~OdometryTransformPublisher();

 private:
  /**
   * @brief Read the parameters from the server
   * @return True if all parameters were parsed correctly, false otherwise
   */
  bool readParameters();

  /**
   * @brief Method to initialize the subscribers in the right namespace for
   *        all the agents
   */
  void initSubscribers();

  /**
   * @brief Method to initialize the publishers for odometry messages
   */
  void initPublisher();

  /**
   * @brief Callback for odometry message for different agents
   * @param[in] odom_msg : odometry message
   * @param[in] agent_id : id number for the agent
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                        const uint64_t agent_id);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::vector<ros::Subscriber> odometry_subs_;
  std::vector<ros::Publisher> odometry_pubs_;
  std::vector<ros::Publisher> transf_W_M_pubs_;
  std::vector<ros::Publisher> transf_M_O_pubs_;

  tf::TransformBroadcaster tf_broadcaster_;

  int num_agents_;
  std::string world_frame_;
  std::string odom_frame_;
  std::string agent_frame_;

  std::vector<tf::Transform> odom_to_world_transforms_;
  std::vector<RandomWalkParams> random_walk_params_;
  std::vector<std::string> agents_ns_;

};  // end class OdometryTransformPublisher

}  // end namespace mrp
