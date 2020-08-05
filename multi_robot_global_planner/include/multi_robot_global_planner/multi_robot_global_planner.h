/**
 * multi_robot_global_planner.h
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for global planning using OMPL and voxblox
 * @date   29.07.2019
 */

#pragma once

#include <geodetic_utils/geodetic_conv.hpp>
#include <glog/logging.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <voxblox_ros/esdf_server.h>

#include "multi_robot_global_planner/interpolators/polynomial_interpolator.h"
#include "multi_robot_global_planner/interpolators/ramp_interpolator.h"
#include "multi_robot_global_planner/voxblox_ompl_rrt.h"

namespace mrp {

/**
 * @brief Main class for global planning. To reduce the computational load,
 * we use one solver for all the agents. The planning is done sequentially,
 * ie agent after the other.
 */
class MultiRobotGlobalPlanner {

public:
  /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
  MultiRobotGlobalPlanner(const ros::NodeHandle &nh,
                          const ros::NodeHandle &nh_private);

  /**
   * @brief Destructor
   */
  ~MultiRobotGlobalPlanner();

  /**
   * @brief Method to read all the additional parameters from the server
   * @return True if all the parameters have been read correctly, false
   * otherwise
   */
  bool readParameters();

private:
  /**
   * @brief Method that initializes ROS-related variables and communications
   */
  void initROS();

  /**
   * @brief Method to read planner parameters from server. If they are not
   * set, use default values.
   */
  void readPlannerParamsFromServer();

  /**
   * @brief Method to read the waypoints each agent has to reach.
   * @return True if the list was parsed successfully. False otherwise.
   */
  bool readWaypointLists();

  /**
   * @brief Callback that trigger the global planner
   * @param[in] req : request for the service
   * @param[out] res : response to the service call
   * @return True if the service was triggered successfully, False otherwise
   */
  bool plannerServiceCallback(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res);

  /**
   * @brief Callback that publishes the paths
   * @param[in] req : request for the service
   * @param[out] res : response to the service call
   * @return True if the service was triggered successfully, False otherwise
   */
  bool publishPathCallback(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res);

  /**
   * @brief Callback for odometry. It stores the odometry for an agent in an
   *        homogeneous transformation
   * @param[in] odom_msg : odometry message to process
   * @param[in] agent_id : agent id the odometry corresponds to
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                        const uint64_t agent_id);

  /**
   * @brief Callback to get the transformation from odometry to map for an
   *        agent.
   * @param[in] transf_msg : the transformation message
   * @param[in[ agent_id : the id of the agent the transformation belongs to
   */
  void odomToMapCallback(
          const geometry_msgs::TransformStampedConstPtr transf_odom_map_msg,
          const uint64_t agent_id);

  /**
   * @brief Callback to get the transformation from map to world for an agent.
   * @param[in] transf_msg : the transformation message
   * @param[in[ agent_id : the id of the agent the transformation belongs to
   */
  void mapToWorldCallback(
          const geometry_msgs::TransformStampedConstPtr transf_map_world_msg,
          const uint64_t agent_id);

  /**
   * @brief Callback when the return home behaviour is triggered
   * @param[in] home_msg : message containing the id of the agent to send home.
   *                       If the message contains -1, all the agents return
   *                       home.
   */
  void returnHomeCallback(const std_msgs::Int16ConstPtr &home_msg);

  /**
   * @brief Callback when one of the agent is told to move to the next
   *        waypoint in the list. If there are no more waypoints to reach,
   *        nothing happens.
   * @param[in] home_msg : message containing the id of the agent
   */
  void moveNextWaypointCallback(const std_msgs::Int16ConstPtr &home_msg);

  /**
   * @brief Callback to replan to current waypoint for one of the agents
   * @param[in] replan_msg : message containing the id of the agent
   */
  void replanToCurrentWaypointCallback(
          const std_msgs::Int16ConstPtr &replan_msg);

  /**
   * @brief Callback for the ROS timer to be executed at fixed time steps
   * @param[in] event : event triggering the planner
   */
  void plannerTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Method to plan the paths of the agents one by one. We iterate
   *        over the agents and plan the paths independently. Collision
   *        checking is also done independently for each agent, so that if a
   *        path for one agent is in collision, only the path of that agent
   *        will be replanned, while the other ones are left unchanged.
   */
  void planOneByOne();

  /**
   * @brief Method to plan the paths for all the agents in one single shot.
   *        However, if one of the paths of the agents is in collision, the
   *        replanning will happen for all of the agents, no matter what. This
   *        function will make the agents stop if one replanning has to happen.
   */
  void planOneShot();

  /**
   * @brief Method to plan the paths for the agents sequentally. So we start
   *        from the first agent, we plan and then we plan for the second one
   *        . The path for the second agent is constrained by the path of the
   *        first one. The third agent will be constrained by the paths for
   *        the first and the second agents. And so on for all the N agents.
   */
  void planConstrainedOneByOne();

  /**
   * @brief Method to plan the path in free known space using ESDF to bring
   *        one agent home
   * @param[in] agent_pose : current pose of the agent
   * @param[in] agent_id : id of the agent to send home
   */
  void planToHomeAgent(const Eigen::Vector4d &agent_pose, const int agent_id);

  /**
   * @brief Method to plan the path in free known space using ESDF to bring
   *        one agent home considering the paths of the other agents
   * @param[in] agent_pose : current pose of the agent
   * @param[in] agent_id : id of the agent to send home
   */
  void planToHomeConstrainedAgent(const Eigen::Vector4d &agent_pose,
          const int agent_id);

  /**
   * @brief Method to check which paths are in collisions
   * @param[in] agent_pose : pose of the agent
   * @param[in] agent_id : agent for which the path has to be computed
   * @param[in] optimistic : whether to do collision checking with TSDFs
   *                         (true) or ESDFs (false). Default: true
   * @return True if at least one of the path is in collision
   */
  bool checkPathsForCollisions(const Eigen::Vector4d &agent_pose,
          const int agent_id,
          bool optimistic = true);

  /**
   * @brief Method to check if the current agent with ID agent_id has a path
   *        that is in collision with another path of another agent with
   *        higher value in the hierarchy. This function will NOT check for
   *        collisions in the map, because we assume we are going towards the
   *        home position and we have already planned in KNOWN FREE space.
   *        The only check is with the paths of the other agents
   * @param[in] agent_pose : pose of the agent
   * @param[in] agent_id : agent for which the path has to be computed
   * @return True if at least one of the path is in collision
   */
  bool checkPathsToHomeForCollisions(const Eigen::Vector4d &agent_pose,
          const int agent_id) const;

  /**
   * @brief Method that computes the global path for an agent to the current
   *        waypoint
   * @param[in] start_pose : pose of the agent to plan from
   * @param[in] agent_id : agent for which the path has to be computed
   * @return True if the computation of the path was successfull
   */
  bool computePathToWaypoint(const Eigen::Vector4d &start_pose,
                             const int agent_id);

  /**
   * @brief Method that computes the global path for an agent to the current
   *        waypoint. The computation of the paths is constrained by all the
   *        paths of the other agents (ie. from 0 to agent_id - 1)
   * @param[in] start_pose : pose of the agent to plan from
   * @param[in] agent_id : agent for which the path has to be computed
   * @return True if the computation of the path was successfull
   */
  bool computeConstrainedPathToWaypoint(const Eigen::Vector4d &start_pose,
                                        const int agent_id);

  /**
   * @brief Method to compute the bounds of the current visible map
   * @param[out] lower_bound : lower bounds of the visible map
   * @param[out] upper_bound : upper bounds of the visible map
   */
  void computeMapBounds(Eigen::Vector3d *lower_bound,
                        Eigen::Vector3d *upper_bound) const;

  /**
   * @brief Method to check if a given robot position is in collision when
   *        adopting an optimistic behaviour
   * @param[in] robot_position : position to be check for collision
   * @param[in] num_agents : agent to be checked
   * @return True if robot is in free or unknown space, False otherwise
   */
  bool checkOptimisticMapCollision(const Eigen::Vector3d &robot_position,
                                   const int num_agents);

  /**
   * @brief Method to compute the distance of a position to the closest
   *        obstacle in the ESDF
   * @param[in] position : position to be checked for the distance
   * @return Distance to the closest obstacle. If the position is in unknown
   *         space, a distance 0.0 is returned
   */
  double getMapDistance(const Eigen::Vector3d& position) const;

  /**
   * @brief Method to get the agent pose in the world frame. First it will
   *        try to look at the TF tree to get the latest update. If it cannot
   *        find one, it will check the transformation that are available in
   *        memory. If these transformations are not initialized, then the
   *        pose will not be retreived and the function will return false.
   * @param[in] agent_id : ID for the agent we are interested in
   * @param[out] pose : pose of the agent in a eigen vector
   * @return True if the pose was retreived successfully, False otherwise
   */
  bool getPoseAgentId(const size_t agent_id, Eigen::Vector4d &pose) const;

  /**
   * @brief Method to check if the current goal for an agent has been reached
   * @param[in] agent_id : id of the agent to be checked
   * @param[in] agent_pose : current pose of the agent in the world frame
   * @return True if goal has been reached, False otherwise
   */
  bool isGoalReachedAgent(const size_t agent_id,
                          const Eigen::Vector4d &agent_pose) const;

  /**
   * @brief Method to publish all the global paths to the agents
   * @return True if the path was successfully published, False otherwise
   */
  bool publishPathsToAgents() const;

  /**
   * @brief Method to publish the global path to a single agent
   * @param[in] agent_id : ID of the agent whose path has to be sent
   * @return True if the path was successfully published, False otherwise
   */
  bool publishPathSingleAgents(const int agent_id) const;

  /**
   * @brief Method to send the stop command to the local planner for a
   *        specific agent
   * @param[in] agent_id : agent id to send the command stop to
   */
  void sendStopCommand(const int agent_id) const;

  /**
   * @brief Method to call the service to stop the local planning for an agent
   * @param[in] agent_id : id agent to be stopped
   */
  void callStopSrvLocalPlanner(const int agent_id);

  /**
   * @brief Method to transform a path in eigen vector format in the right way
   * @param[in] path_eigen_vector : path to be stored in GlobalPath format
   * @return The path in GlobalPath format
   */
  GlobalPath getGlobalPathFromEigenVector(
         const mav_msgs::EigenTrajectoryPoint::Vector &path_eigen_vector) const;

  /**
   * @brief Method to transform a path made of positions in std vector format
   *        in the right way
   * @param[in] path_std_vector  : path to be stored in GlobalPath format
   * @param[in] start_yaw  : yaw for the first position
   * @return The path in GlobalPath format
   */
  GlobalPath getGlobalPathFromStdVector(
          const std::vector<Eigen::Vector3d> &path_std_vector,
          const double start_yaw) const;

  /**
   * @brief Method to check if the return home behaviour for an agent has
   *        been triggered
   * @param[in] agent_id : agent id to check
   * @return True if the agent has been sent home, False otherwise
   */
  inline bool isAgentSentHome(const int agent_id) const;

  /**
   * @brief Method to check if we have to force replanning after reaching
   *        thershold of iterations
   * @return True if the replanning must be force, False otherwise
   */
  bool checkForceReplanning();

  /**
   * @brief Method to publish the marker of the waypoints to plan to
   */
  void publishMarkerWaypoints() const;

  /**
   * @brief Method to publish the marker for visualizing the global paths for
   * the different agents
   */
  void publishMarkerPaths() const;

protected:
  // ROS Variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer planner_srv_;
  ros::ServiceServer path_pub_srv_;

  ros::Publisher path_marker_pub_;
  ros::Publisher waypoints_marker_pub_;
  std::vector<ros::Publisher> global_paths_pubs_;

  ros::Subscriber return_home_sub_;
  ros::Subscriber move_next_waypoint_sub_;
  ros::Subscriber replan_to_current_waypoint_sub_;
  std::vector<ros::Subscriber> odometry_subs_;
  std::vector<ros::Subscriber> transf_odom_map_subs_;
  std::vector<ros::Subscriber> transf_map_world_subs_;

  ros::Timer planning_timer_;
  ros::CallbackQueue planning_queue_;
  ros::AsyncSpinner planning_spinner_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;

  // Planner
  std::unique_ptr<VoxbloxOmplRrt> rrt_;

  // Interpolator
  std::unique_ptr<PolynomialInterpolator> polynomial_interpolator_;
  std::unique_ptr<RampInterpolator> ramp_interpolator_;

  // Geodetic coordinate conversion (from lat/lon to Cartesian ENU).
  geodetic_converter::GeodeticConverter geodetic_converter_;

  // Transformation listener - we only need one
  tf::TransformListener tf_listener_;

  // Parameters
  GlobalPlannerParams params_;
  DynamicParams dynamic_params_;
  int num_agents_;
  double timer_dt_;
  double clock_replanning_;
  int iter_timer_;
  bool perform_planning_;

  // GPS/ENU coordinates.
  std::string coordinate_type_;

  // Reference frames names (only the base of the name)
  std::string world_frame_;
  std::string map_frame_;
  std::string odometry_frame_;
  std::string agent_frame_;

  // Global waypoint lists to be reached for all the agents
  std::vector<WaypointsList> waypoints_lists_;

  // Storage for the global paths of all the agents
  std::vector<double> robot_radii_;
  std::vector<GlobalPath> global_paths_;

  // Storage for the current waypoint number in the waypoint list. It stores
  // the current waypoint list index to plan to
  std::vector<uint64_t> waypoint_nums_;

  // Storage of checks: it tells us if all the agents have valid paths
  std::vector<bool> valid_paths_;
  std::vector<bool> reached_all_goals_;
  std::vector<bool> move_next_waypoint_;

  // Storage for the starting positions of all the agents, in case the return
  // home is triggered
  std::vector<Eigen::Vector3d> home_positions_;
  std::vector<bool> home_triggered_;

  // Storages for the transformations
  // Reference frame convention for each agent:
  // - W : world
  // - M : map
  // - O : odom
  // - A : agent
  // Convention: T_B_A --> from A to B
  std::vector<Eigen::Matrix4d> T_O_A_; // this is from odometry information
  std::vector<Eigen::Matrix4d> T_W_M_; // this is from pose graph backend
  std::vector<Eigen::Matrix4d> T_M_O_; // this is from pose graph backend

  std::vector<bool> transformations_initialized_O_A_;
  std::vector<bool> transformations_initialized_M_O_;
  std::vector<bool> transformations_initialized_W_M_;

}; // end class MultiRobotGlobalPlanner

} // end namespace mrp
