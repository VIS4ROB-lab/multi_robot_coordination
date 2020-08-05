/**
 * @author Luca Bartolomei, V4RL
 * @date   31.08.2019
 */

#include "multi_robot_simulation/vins_initializer.h"

#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace mrp {

VinsInitializer::VinsInitializer(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private,
                                 const int agent_id)
    : nh_(nh),
      nh_private_(nh_private),
      has_odometry_(false),
      agent_id_(agent_id) {
  // Get parameters
  std::string ns("vins_initializer_node_" + std::to_string(agent_id_) + "/");

  double n_sec_wait;
  CHECK(nh_.getParam(ns + "n_sec_wait", n_sec_wait));

  double hovering_height, delta_d, delta_h, delta_yaw;
  CHECK(nh_.getParam(ns + "hovering_height", hovering_height));
  CHECK(nh_.getParam(ns + "delta_d", delta_d));
  CHECK(nh_.getParam(ns + "delta_h", delta_h));
  CHECK(nh_.getParam(ns + "delta_yaw", delta_yaw));
  delta_yaw *= M_PI / 180.0;

  double v_max, a_max, v_max_yaw, a_max_yaw, sampling_dt;
  if (!nh_.getParam(ns + "v_max", v_max)) {
    v_max = 2.0;
  }
  if (!nh_.getParam(ns + "a_max", a_max)) {
    a_max = 2.0;
  }
  if (!nh_.getParam(ns + "v_max_yaw", v_max_yaw)) {
    v_max = 2.0;
  }
  if (!nh_.getParam(ns + "a_max_yaw", a_max_yaw)) {
    a_max = 2.0;
  }
  if (!nh_.getParam(ns + "sampling_dt", sampling_dt)) {
    sampling_dt = 2.0;
  }

  // Set up ros communication
  odom_sub_ = nh_.subscribe("odometry_" + std::to_string(agent_id_), 10,
                            &mrp::VinsInitializer::odometryCallback, this);
  trajectory_cmd_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 100);

  // Wait until you get odometry
  ros::Duration(n_sec_wait).sleep();
  while (!has_odometry_) {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  ROS_INFO_STREAM("[Vins Initializer] Ready to initialize agent " << agent_id_);

  // Waypoints for initialization in ground truth frame (plus, landing)
  std::vector<Eigen::Vector3d> waypoints(
      8, Eigen::Vector3d(odometry_.position_W.x(), odometry_.position_W.y(),
                         hovering_height));

  waypoints[1] += Eigen::Vector3d(delta_d, 0.0, delta_h);
  waypoints[2] += Eigen::Vector3d(-delta_d, 0.0, -delta_h);
  waypoints[4] += Eigen::Vector3d(delta_d / 3.0, delta_d, delta_h);
  waypoints[6] += Eigen::Vector3d(-delta_d / 3.0, -delta_d, -delta_h);

  std::vector<double> yaws(8, odometry_.getYaw());
  yaws[1] -= delta_yaw;
  yaws[2] += delta_yaw;
  yaws[4] -= delta_yaw;
  yaws[6] += delta_yaw;

  // Interpolate trajectory
  mav_trajectory_generation::Vertex::Vector vertices, vertices_yaw;
  const int dimension = 3;
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  const int derivative_to_optimize_yaw =
      mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

  mav_trajectory_generation::Vertex vertex(dimension), vertex_yaw(1);
  vertex.makeStartOrEnd(waypoints[0], derivative_to_optimize);

  double yaw_old = yaws[0];
  vertex_yaw.makeStartOrEnd(yaws[0], derivative_to_optimize_yaw);

  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    mav_trajectory_generation::Vertex vertex_m(dimension), vertex_yaw_m(1);
    vertex_m.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);

    if (std::abs(yaws[i] + 2 * M_PI - yaw_old) < std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] + 2 * M_PI;
    } else if (std::abs(yaws[i] - 2 * M_PI - yaw_old) <
               std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] - 2 * M_PI;
    } else {
      yaw_old = yaws[i];
    }

    vertex_yaw_m.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION, yaw_old);
    vertices.push_back(vertex_m);
    vertices_yaw.push_back(vertex_yaw_m);
  }

  vertex.makeStartOrEnd(waypoints.back(), derivative_to_optimize);
  if (std::abs(yaws.back() + 2 * M_PI - yaw_old) <
      std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() + 2 * M_PI;
  } else if (std::abs(yaws.back() - 2 * M_PI - yaw_old) <
             std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() - 2 * M_PI;
  } else {
    yaw_old = yaws.back();
  }
  vertex_yaw.makeStartOrEnd(yaw_old, derivative_to_optimize_yaw);
  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  // Step 3: obtain interpolated trajectory
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max);
  std::vector<double> segment_times_yaw{
      mav_trajectory_generation::estimateSegmentTimes(vertices_yaw, v_max_yaw,
                                                      a_max_yaw)};

  for (int i = 0; i < segment_times.size(); ++i) {
    if (segment_times_yaw[i] > segment_times[i])
      segment_times[i] = segment_times_yaw[i];

    if (segment_times[i] < sampling_dt) segment_times[i] = sampling_dt;
  }

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  mav_trajectory_generation::PolynomialOptimization<N> opt_yaw(1);
  opt_yaw.setupFromVertices(vertices_yaw, segment_times,
                            derivative_to_optimize_yaw);
  opt_yaw.solveLinear();

  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory trajectory_yaw;
  mav_trajectory_generation::Trajectory trajectory_with_yaw;
  opt.getTrajectory(&trajectory);
  opt_yaw.getTrajectory(&trajectory_yaw);
  trajectory.getTrajectoryWithAppendedDimension(trajectory_yaw,
                                                &trajectory_with_yaw);

  mav_msgs::EigenTrajectoryPoint::Vector states;
  mav_trajectory_generation::sampleWholeTrajectory(trajectory_with_yaw,
                                                   sampling_dt, &states);

  // Create message for controller
  trajectory_msgs::MultiDOFJointTrajectory cmd_trajectory;
  cmd_trajectory.header.seq = 0;
  cmd_trajectory.header.frame_id = frame_id_;
  cmd_trajectory.header.stamp = ros::Time::now();

  for (int i = 0; i < states.size(); ++i) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint cmd_point;
    cmd_point.transforms.resize(1);
    cmd_point.transforms[0].translation.x = states[i].position_W.x();
    cmd_point.transforms[0].translation.y = states[i].position_W.y();
    cmd_point.transforms[0].translation.z = states[i].position_W.z();

    cmd_point.transforms[0].rotation.x = states[i].orientation_W_B.x();
    cmd_point.transforms[0].rotation.y = states[i].orientation_W_B.y();
    cmd_point.transforms[0].rotation.z = states[i].orientation_W_B.z();
    cmd_point.transforms[0].rotation.w = states[i].orientation_W_B.w();

    cmd_point.velocities.resize(1);
    cmd_point.velocities[0].linear.x = states[i].velocity_W.x();
    cmd_point.velocities[0].linear.y = states[i].velocity_W.y();
    cmd_point.velocities[0].linear.z = states[i].velocity_W.z();

    cmd_point.velocities[0].angular.x = states[i].angular_velocity_W.x();
    cmd_point.velocities[0].angular.y = states[i].angular_velocity_W.y();
    cmd_point.velocities[0].angular.z = states[i].angular_velocity_W.z();

    cmd_point.accelerations.resize(1);
    cmd_point.accelerations[0].linear.x = states[i].acceleration_W.x();
    cmd_point.accelerations[0].linear.y = states[i].acceleration_W.y();
    cmd_point.accelerations[0].linear.z = states[i].acceleration_W.z();

    cmd_point.accelerations[0].angular.x = states[i].angular_acceleration_W.x();
    cmd_point.accelerations[0].angular.y = states[i].angular_acceleration_W.y();
    cmd_point.accelerations[0].angular.z = states[i].angular_acceleration_W.z();

    cmd_point.time_from_start =
        ros::Duration(states[i].time_from_start_ns * 1e-9);

    cmd_trajectory.points.push_back(cmd_point);
  }

  // Publish the controller command
  trajectory_cmd_pub_.publish(cmd_trajectory);
  ROS_INFO_STREAM(
      "[Vins Initializer] Sent trajectory for initialization of "
      "agent "
      << agent_id_);
}

VinsInitializer::~VinsInitializer() {}

void VinsInitializer::odometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  ROS_INFO_ONCE("[Vins Initializer] Initialize odometry");

  has_odometry_ = true;
  frame_id_ = odom_msg->header.frame_id;
  mav_msgs::eigenOdometryFromMsg(*odom_msg, &odometry_);
}

}  // end namespace mrp