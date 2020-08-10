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
 * @brief  Main class for reading a command in the odometry frame and
 *         republish it in the controller reference frame
 * @date   03.09.2019
 */

#pragma once

#include <glog/logging.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace mrp {

class CommandInterceptor {
 public:
  /**
   * @brief Constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   * @param[in] agent_id : ID of the agent
   */
  CommandInterceptor(const ros::NodeHandle &nh,
                     const ros::NodeHandle &nh_private, const int agent_id);

  /**
   * @brief Destructor
   */
  virtual ~CommandInterceptor();

 private:
  /**
   * @brief Callback to read the command
   * @param[in] traj_msg : command to be processed
   */
  void commandTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &traj_msg);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber trajectory_cmd_sub_;
  ros::Publisher trajectory_cmd_pub_;
  tf::TransformListener tf_listener_;

  int agent_id_;
  std::string controller_frame_id_;
  std::string state_frame_id_;
  std::string body_frame_id_;
  Eigen::Matrix4d T_C_O_;  // Transformation from odometry to controller frame
  bool has_valid_transformation_;
};  // end class command interceptor

}  // end namespace mrp
