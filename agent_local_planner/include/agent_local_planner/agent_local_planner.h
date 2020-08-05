/**
 * agent_local_planner.h
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for local planning using EWOK and Octomap
 * @date   02.08.2019
 */

#pragma once

#include <mutex>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>
#include <glog/logging.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "agent_local_planner/polynomial_interpolator.h"

#define POW 6
#define NOPT 10

#define DEBUG true

namespace mrp {

class AgentLocalPlanner {
 public:
  /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
  AgentLocalPlanner(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private);

  /**
   * @brief Destructor
   */
  ~AgentLocalPlanner();

 private:
  /**
   * @brief Method that initializes the ROS communication
   */
  void initROS();

  /**
   * @brief Series of callbacks
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg);

  void globalPathCallback(const nav_msgs::PathConstPtr &path_msg);

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg);

  bool stopServiceCallback(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res);

  void plannerTimerCallback(const ros::TimerEvent &e);

  void commandPublishTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Method to send a stop command to the MPC. It resets the local
   *        planner as well
   */
  void sendStopCommand() const;

  /**
   * @brief Method that resets the local planner when stop command is sent
   */
  void resetLocalPlanner();

  /**
   * @brief Method to get the command to get to the control point t seconds
   *        along the spline
   * @param[in] t : query time
   * @param[out] command_trajectory : trajectory command to move along the
   * s                                pline
   * @return True if a valid command was found, False otherwise
   */
  bool getTrajectoryPoint(
      double t, mav_msgs::EigenTrajectoryPoint &command_trajectory) const;

  /**
   * @brief Method to update the transformation from odometry to drifted
   *        odometry frames
   */
  void updateTransformToDriftedOdom();

  /**
   * @brief Method that interpolates the yaw of the local path
   *        to avoid problems with the spline
   */
  void getYawsAlongLocalPath();

  /**
   * @brief Method to colorize the occupancy map depending on the height of
   *        the marker in the map
   * @param[in] z : current height of the marker
   * @param[in] min_z : min height in the color map (from map boundaries)
   * @param[in] max_z : max height in the color map (from map boundaries)
   * @return Percentage for the color to be used by the 'percentToColor'
   *         function. It is basically the percentage position of the marker
   *         between min_z and max_z
   */
  inline double colorizeMapByHeight(double z, double min_z, double max_z) const;

  /**
   * @brief Method that returns the color from the percentual position of the
   *        marker w.r.t. the height boundaries
   * @param[in] h : percentage computed by 'colorizeMapByHeight'
   * @return The color of the marker
   */
  std_msgs::ColorRGBA percentToColor(double h) const;

 protected:
  // ROS variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odometry_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber pcl_sub_;

  ros::Publisher occupied_marker_pub_;
  ros::Publisher free_marker_pub_;
  ros::Publisher distance_marker_pub_;
  ros::Publisher global_trajectory_marker_pub_;
  ros::Publisher optimal_trajectory_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher command_visual_pub_;

  ros::ServiceServer stop_srv_;

  ros::Timer planning_timer_;
  ros::CallbackQueue planning_queue_;
  ros::AsyncSpinner planning_spinner_;

  ros::Timer command_publishing_timer_;
  ros::CallbackQueue command_publishing_queue_;
  ros::AsyncSpinner command_publishing_spinner_;

  tf::TransformListener tf_listener_;
  Eigen::Matrix4d T_w_o_;  // Transformation world to odom when global
                           // path is received
  Eigen::Matrix4d T_odrifted_o_;

  // Check variables
  bool map_initialized_;
  bool got_global_path_;
  bool stopped_;  // Variable when the local planner receives an hard stop

  // Parameters
  int agent_id_;
  double map_resolution_;

  double vel_max_;
  double vel_max_yaw_;
  double acc_max_;
  double acc_max_yaw_;
  double sampling_dt_;

  double dt_;
  double dt_commands_;
  double prediction_horizon_mpc_;
  double local_goal_distance_;

  // Visualization stuff
  double scale_factor_visualization_;
  double min_z_;
  double max_z_;

  // Spline parameters
  int num_opt_points_;
  double distance_threshold_;

  std::string odometry_frame_;
  std::string world_frame_;
  mav_msgs::EigenOdometry odometry_;
  bool has_odom_to_world_;

  // Mutex for path and command buffer access
  std::recursive_mutex commands_mutex_;
  std::recursive_mutex map_mutex_;

  // EWOK
  ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb_;
  ewok::PolynomialTrajectory3D<NOPT>::Ptr poly_traj_;
  ewok::UniformBSpline3DOptimization<POW>::Ptr spline_optimization_;
  ewok::UniformBSpline3D<POW, double>::Ptr b_spline_;

  // Auxiliaries for planning
  ros::Time init_time_;
  Eigen::Vector3d goal_;

  bool inserted_goal_;
  bool back_to_position_hold_called_;

  // Storage for the local path
  LocalPath path_queue_;
  LocalPath interpolated_path_queue_;
  int path_index_;

  // Interpolator
  std::unique_ptr<mrp::PolynomialInterpolator> poly_interpolator_;

  // Debug
  std::ofstream log_file_;

};  // end class AgentLocalPlanner

}  // end namespace mrp