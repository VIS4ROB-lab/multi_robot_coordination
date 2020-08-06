/**
 * @author Luca Bartolomei, V4RL
 * @date   31.08.2019
 */

#include "multi_robot_simulation/command_interceptor.h"

#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/default_topics.h>

namespace mrp {

CommandInterceptor::CommandInterceptor(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &nh_private,
                                       const int agent_id)
    : nh_(nh),
      nh_private_(nh_private),
      agent_id_(agent_id),
      T_C_O_(Eigen::Matrix4d::Identity()),
      has_valid_transformation_(false) {
  // Get parameters
  std::string ns("/command_interceptor_node_" + std::to_string(agent_id_) +
                 "/");
  CHECK(nh_private_.getParam(ns + "controller_frame_id", controller_frame_id_))
      << "Controller reference frame name not specified (param: " << ns
      << "controller_frame_id)";
  CHECK(nh_private_.getParam(ns + "state_frame_id", state_frame_id_))
      << "State reference frame name not specified (param: " << ns
      << "state_frame_id)";
  CHECK(nh_private_.getParam(ns + "body_frame_id", body_frame_id_))
      << "State reference frame name not specified (param: " << ns
      << "body_frame_id)";

  ROS_INFO_STREAM("[Command Interceptor] Set target reference frame to "
                  << controller_frame_id_);
  ROS_INFO_STREAM("[Command Interceptor] Set state reference frame to "
                  << state_frame_id_);
  ROS_INFO_STREAM("[Command Interceptor] Set body reference frame to "
                  << body_frame_id_);

  // Initialize subscriber and publisher
  trajectory_cmd_sub_ =
      nh_.subscribe("command/trajectory_odom", 10,
                    &mrp::CommandInterceptor::commandTrajectoryCallback, this);
  trajectory_cmd_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
}

CommandInterceptor::~CommandInterceptor() {}

void CommandInterceptor::commandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &traj_msg) {
  // Update the transformation first
  tf::StampedTransform tf_O_S, tf_C_S;
  ROS_INFO_STREAM("[Command Interceptor] Transforming the commands from "
                  << traj_msg->header.frame_id << " to "
                  << controller_frame_id_);
  try {
    tf_listener_.lookupTransform(traj_msg->header.frame_id, state_frame_id_,
                                 ros::Time(0), tf_O_S);
    tf_listener_.lookupTransform(controller_frame_id_, body_frame_id_,
                                 ros::Time(0), tf_C_S);

    Eigen::Quaterniond q_O_S;
    tf::quaternionTFToEigen(tf_O_S.getRotation(), q_O_S);
    Eigen::Matrix4d T_O_S = Eigen::Matrix4d::Identity();
    T_O_S.block<3, 3>(0, 0) = q_O_S.normalized().toRotationMatrix();
    T_O_S.block<3, 1>(0, 3) =
        Eigen::Vector3d(tf_O_S.getOrigin().getX(), tf_O_S.getOrigin().getY(),
                        tf_O_S.getOrigin().getZ());

    Eigen::Quaterniond q_C_S;
    tf::quaternionTFToEigen(tf_C_S.getRotation(), q_C_S);
    Eigen::Matrix4d T_C_S = Eigen::Matrix4d::Identity();
    T_C_S.block<3, 3>(0, 0) = q_C_S.normalized().toRotationMatrix();
    T_C_S.block<3, 1>(0, 3) =
        Eigen::Vector3d(tf_C_S.getOrigin().getX(), tf_C_S.getOrigin().getY(),
                        tf_C_S.getOrigin().getZ());

    // Concatenate the transformations
    T_C_O_ = T_C_S * T_O_S.inverse();

    /** In this part, we try to get directly without transformation
     * concatenation
     tf::StampedTransform tf_C_O;
     tf_listener_.lookupTransform(controller_frame_id_,
                                 traj_msg->header.frame_id,
                                 ros::Time(0), tf_C_O);
    Eigen::Quaterniond q_C_O;
    tf::quaternionTFToEigen(tf_C_O.getRotation(), q_C_O);

    T_C_O_.block<3,3>(0,0) = q_C_O.normalized().toRotationMatrix();
    T_C_O_.block<3,1>(0,3) = Eigen::Vector3d(
            tf_C_O.getOrigin().getX(),
            tf_C_O.getOrigin().getY(),
            tf_C_O.getOrigin().getZ());
    T_C_O_.block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;
    **/

    // Set flag
    has_valid_transformation_ = true;

  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM(
        "[Command Interceptor] Error getting TF transform from sensor "
        "data: "
        << ex.what());
    if (!has_valid_transformation_) {
      ROS_ERROR("[Command Interceptor] Don't have a valid transformation yet");
      return;
    }
  }

  // Create the trajectory
  trajectory_msgs::MultiDOFJointTrajectory command_trajectory;
  command_trajectory.header.frame_id = controller_frame_id_;
  command_trajectory.header.stamp = ros::Time::now();
  command_trajectory.header.seq = 0;

  for (size_t i = 0; i < traj_msg->points.size(); ++i) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
    Eigen::Vector4d position(
        T_C_O_ *
        Eigen::Vector4d(traj_msg->points[i].transforms[0].translation.x,
                        traj_msg->points[i].transforms[0].translation.y,
                        traj_msg->points[i].transforms[0].translation.z, 1.0));

    Eigen::Quaterniond q_o_p;
    q_o_p.x() = traj_msg->points[i].transforms[0].rotation.x;
    q_o_p.y() = traj_msg->points[i].transforms[0].rotation.y;
    q_o_p.z() = traj_msg->points[i].transforms[0].rotation.z;
    q_o_p.w() = traj_msg->points[i].transforms[0].rotation.w;

    Eigen::Matrix3d R_o_p(q_o_p.normalized().toRotationMatrix());
    Eigen::Matrix3d R_c_p(T_C_O_.block<3, 3>(0, 0) * R_o_p);
    tf::Quaternion q_c_p;
    tf::quaternionEigenToTF(Eigen::Quaterniond(R_c_p), q_c_p);
    double drifted_yaw(tf::getYaw(q_c_p));

    trajectory_point.transforms.resize(1);
    tf::vectorEigenToMsg(position.head(3),
                         trajectory_point.transforms[0].translation);
    trajectory_point.transforms[0].rotation =
        tf::createQuaternionMsgFromYaw(drifted_yaw);
    trajectory_point.time_from_start = traj_msg->points[i].time_from_start;
    command_trajectory.points.push_back(trajectory_point);
  }

  trajectory_cmd_pub_.publish(command_trajectory);
}

}  // end namespace mrp
