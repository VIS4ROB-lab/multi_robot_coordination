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
