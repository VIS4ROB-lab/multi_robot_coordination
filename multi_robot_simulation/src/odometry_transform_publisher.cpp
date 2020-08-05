/**
 * @author Luca Bartolomei, V4RL
 * @date   31.07.2019
 */

#include "multi_robot_simulation/odometry_transform_publisher.h"

#include <random>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Eigen>

namespace mrp {

OdometryTransformPublisher::OdometryTransformPublisher(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  if (readParameters()) {
    initSubscribers();
    initPublisher();
  }

  // Just idle a bit to make the gazebo simulation start
  ros::Duration(2.0).sleep();
}

OdometryTransformPublisher::~OdometryTransformPublisher() {}

bool OdometryTransformPublisher::readParameters() {
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

  // Read and create initial reference frames
  odom_to_world_transforms_.reserve(num_agents_);
  random_walk_params_.reserve(num_agents_);
  agents_ns_.reserve(num_agents_);

  for (size_t id = 0; id < num_agents_; ++id) {
    std::string agent_name = "agent_" + std::to_string(id);

    // Get the agent namespaces
    std::string agent_namespace = agent_name + "/namespace";
    std::string ns;
    load_success &= nh_private_.getParam(agent_namespace, ns);
    CHECK(load_success) << "Failed to read agent namespace";
    agents_ns_.push_back(ns);

    // Position of the odometry frame for the current agent
    const std::string odom_pos_name = agent_name + "/position";
    XmlRpc::XmlRpcValue input_pos;
    load_success &= nh_private_.getParam(odom_pos_name, input_pos);
    CHECK(load_success) << "Failed to read odometry frame position for agent "
                        << id;

    const std::string odom_orient_name = agent_name + "/yaw";
    double input_orient;
    load_success &= nh_private_.getParam(odom_orient_name, input_orient);
    input_orient *= M_PI / 180.0;  // to radiants
    CHECK(load_success) << "Failed to read odom frame orientation for agent "
                        << id;

    // Store the transformation for the current agent
    tf::Transform transform;

    tf::Vector3 position;
    position.setX(static_cast<double>(input_pos[0]));
    position.setY(static_cast<double>(input_pos[1]));
    position.setZ(static_cast<double>(input_pos[2]));
    transform.setOrigin(position);

    tf::Quaternion quaternion = tf::createQuaternionFromYaw(input_orient);
    transform.setRotation(quaternion);
    odom_to_world_transforms_.push_back(transform);

    // Get the random walk parameters and store them
    std::string random_walk_mean_name = agent_name + "/random_walk_mean_pos";
    RandomWalkParams random_walk_params;
    load_success &= nh_private_.getParam(random_walk_mean_name,
                                         random_walk_params.mean_pos);
    CHECK(load_success) << "Failed to read random walk mean for agent " << id;

    std::string random_walk_cov_name = agent_name + "/random_walk_cov_pos";
    load_success &=
        nh_private_.getParam(random_walk_cov_name, random_walk_params.cov_pos);
    CHECK(load_success) << "Failed to read random walk covariance for agent "
                        << id;

    random_walk_mean_name = agent_name + "/random_walk_mean_ang";
    load_success &= nh_private_.getParam(random_walk_mean_name,
                                         random_walk_params.mean_ang);
    CHECK(load_success) << "Failed to read random walk mean ang for agent "
                        << id;

    random_walk_cov_name = agent_name + "/random_walk_cov_ang";
    load_success &=
        nh_private_.getParam(random_walk_cov_name, random_walk_params.cov_ang);
    CHECK(load_success) << "Failed to read random walk covariance ang for "
                           "agent "
                        << id;
    random_walk_params_.push_back(random_walk_params);
  }

  return true;
}

void OdometryTransformPublisher::initSubscribers() {
  // Set up all callbacks for odometry info for all the agents
  odometry_subs_.reserve(num_agents_);

  for (uint64_t id = 0; id < num_agents_; ++id) {
    std::string odom_ns = agents_ns_[id] + "/ground_truth/odometry";

    ros::Subscriber tmp_sub_odom = nh_.subscribe<nav_msgs::Odometry>(
        odom_ns, 10,
        boost::bind(&OdometryTransformPublisher::odometryCallback, this, _1,
                    id));
    odometry_subs_.push_back(tmp_sub_odom);
  }
}

void OdometryTransformPublisher::initPublisher() {
  odometry_pubs_.reserve(num_agents_);

  for (uint64_t id = 0; id < num_agents_; ++id) {
    std::string odometry_topic = agents_ns_[id] + "/odometry";
    ros::Publisher tmp_pub_odom =
        nh_.advertise<nav_msgs::Odometry>(odometry_topic, 10);
    odometry_pubs_.push_back(tmp_pub_odom);
  }
}

void OdometryTransformPublisher::odometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg, const uint64_t agent_id) {
  // Get the eigen transform from the odometry message
  tf::Quaternion quaternion_tf;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quaternion_tf);
  Eigen::Quaterniond quaternion_eigen;
  tf::quaternionTFToEigen(quaternion_tf, quaternion_eigen);

  Eigen::Matrix4d T_w_a;
  T_w_a.block(0, 3, 3, 1) << odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
  T_w_a.block(0, 0, 3, 3) << quaternion_eigen.normalized().toRotationMatrix();
  T_w_a.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;

  // Corrupt the world to odom transformation with random walk
  std::default_random_engine generator(std::time(0));
  std::normal_distribution<double> distribution_pos(
      random_walk_params_[agent_id].mean_pos,
      random_walk_params_[agent_id].cov_pos);
  std::normal_distribution<double> distribution_ang(
      random_walk_params_[agent_id].mean_ang,
      random_walk_params_[agent_id].cov_ang);

  tf::Transform odom_to_world_noisy_;
  odom_to_world_noisy_.getOrigin().setX(
      odom_to_world_transforms_[agent_id].getOrigin().getX() +
      distribution_pos(generator));

  odom_to_world_noisy_.getOrigin().setY(
      odom_to_world_transforms_[agent_id].getOrigin().getY() +
      distribution_pos(generator));

  odom_to_world_noisy_.getOrigin().setZ(
      odom_to_world_transforms_[agent_id].getOrigin().getZ() +
      distribution_pos(generator));

  tf::Quaternion quaternion = tf::createQuaternionFromYaw(
      tf::getYaw(odom_to_world_transforms_[agent_id].getRotation()) +
      distribution_ang(generator));
  odom_to_world_noisy_.setRotation(quaternion);

  // Store the noisy transform to simulate random walk
  odom_to_world_transforms_[agent_id] = odom_to_world_noisy_;

  // Get the transformation world to odom for agent id
  tf::quaternionTFToEigen(odom_to_world_noisy_.getRotation(), quaternion_eigen);
  Eigen::Matrix4d T_w_o;
  T_w_o.block(0, 3, 3, 1) << odom_to_world_noisy_.getOrigin().getX(),
      odom_to_world_noisy_.getOrigin().getY(),
      odom_to_world_noisy_.getOrigin().getZ();
  T_w_o.block(0, 0, 3, 3) << quaternion_eigen.normalized().toRotationMatrix();
  T_w_o.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;

  // Get the transformation between odom and agent
  Eigen::Matrix4d T_o_a = T_w_o.inverse() * T_w_a;

  // Publish this transformation as a TF
  tf::Transform transform_o_a;

  tf::Vector3 position_o_a;
  tf::vectorEigenToTF(T_o_a.block(0, 3, 3, 1), position_o_a);
  transform_o_a.setOrigin(position_o_a);

  tf::Quaternion quaternion_o_a;
  tf::quaternionEigenToTF(
      Eigen::Quaterniond(Eigen::Matrix3d(T_o_a.block(0, 0, 3, 3))),
      quaternion_o_a);
  transform_o_a.setRotation(quaternion_o_a);

  ROS_INFO_ONCE("Published first transformation");
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform_o_a, ros::Time::now(),
                           odom_frame_ + "_" + std::to_string(agent_id),
                           agent_frame_ + "_" + std::to_string(agent_id)));

  // Publish the transformation world to odom
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(odom_to_world_noisy_, ros::Time::now(), world_frame_,
                           odom_frame_ + "_" + std::to_string(agent_id)));

  // Publish odometry message odom to agent
  nav_msgs::Odometry odometry_msg_o_a;
  odometry_msg_o_a.header.frame_id =
      odom_frame_ + "_" + std::to_string(agent_id);
  odometry_msg_o_a.header.seq = 0;
  odometry_msg_o_a.header.stamp = ros::Time::now();
  odometry_msg_o_a.child_frame_id =
      agent_frame_ + "_" + std::to_string(agent_id);

  odometry_msg_o_a.pose.pose.position.x = position_o_a.x();
  odometry_msg_o_a.pose.pose.position.y = position_o_a.y();
  odometry_msg_o_a.pose.pose.position.z = position_o_a.z();

  odometry_msg_o_a.pose.pose.orientation.x = quaternion_o_a.x();
  odometry_msg_o_a.pose.pose.orientation.y = quaternion_o_a.y();
  odometry_msg_o_a.pose.pose.orientation.z = quaternion_o_a.z();
  odometry_msg_o_a.pose.pose.orientation.w = quaternion_o_a.w();

  // Add also the twist
  Eigen::Vector3d linear_velocity(odom_msg->twist.twist.linear.x,
                                  odom_msg->twist.twist.linear.y,
                                  odom_msg->twist.twist.linear.z);
  Eigen::Vector3d angular_velocity(odom_msg->twist.twist.angular.x,
                                   odom_msg->twist.twist.angular.y,
                                   odom_msg->twist.twist.angular.z);

  Eigen::Vector3d transformed_lin_vel(T_w_o.block(0, 0, 3, 3).transpose() *
                                      linear_velocity);
  Eigen::Vector3d transformed_ang_vel(T_w_o.block(0, 0, 3, 3).transpose() *
                                      angular_velocity);

  odometry_msg_o_a.twist.twist.linear.x = transformed_lin_vel.x();
  odometry_msg_o_a.twist.twist.linear.y = transformed_lin_vel.y();
  odometry_msg_o_a.twist.twist.linear.z = transformed_lin_vel.z();

  odometry_msg_o_a.twist.twist.angular.x = transformed_ang_vel.x();
  odometry_msg_o_a.twist.twist.angular.y = transformed_ang_vel.y();
  odometry_msg_o_a.twist.twist.angular.z = transformed_ang_vel.z();

  ROS_INFO_ONCE("Published first odometry message");
  odometry_pubs_[agent_id].publish(odometry_msg_o_a);
}

}  // end namespace mrp
