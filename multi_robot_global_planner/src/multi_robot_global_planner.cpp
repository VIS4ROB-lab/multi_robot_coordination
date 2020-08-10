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
 * multi_robot_global_planner.cpp
 * @author Luca Bartolomei, V4RL
 * @date   29.07.2019
 */

#include "multi_robot_global_planner/multi_robot_global_planner.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>

#include "multi_robot_global_planner/utils/color.h"

namespace mrp {

MultiRobotGlobalPlanner::MultiRobotGlobalPlanner(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      planning_spinner_(1, &planning_queue_),
      voxblox_server_(nh_, nh_private_),
      num_agents_(1),
      iter_timer_(0),
      perform_planning_(false),
      world_frame_("world"),
      map_frame_("map"),
      odometry_frame_("odom"),
      agent_frame_("agent") {
  // Get the parameters
  readParameters();

  // Initialize Voxblox maps
  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  ROS_INFO(
      "[MR Global Planner] Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // For the traversability radius in voxblox, use the biggest one to be
  // conservative
  auto max_radius =
      std::max_element(std::begin(robot_radii_), std::end(robot_radii_));

  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Setting traversability radius to "
                    << *max_radius);
  }

  voxblox_server_.setTraversabilityRadius(static_cast<float>(*max_radius));
  voxblox_server_.setSliceLevel(params_.planning_height);

  // Additional checks for voxblox if we want to add visual output
  if (params_.visualize) {
    voxblox_server_.setPublishSlices(true);
    voxblox_server_.publishSlices();
    voxblox_server_.publishTraversable();
  }

  // FIXME Uniform this
  switch (params_.planning_strategy) {
    case PlanningStrategy::OneByOne:
      rrt_ = std::unique_ptr<mrp::VoxbloxOmplRrt>(
          new mrp::VoxbloxOmplRrt(nh_, nh_private_));
      break;
    case PlanningStrategy::OneShot:
      rrt_ = std::unique_ptr<mrp::VoxbloxOmplRrt>(
          new mrp::VoxbloxOmplRrt(nh_, nh_private_, num_agents_));
      break;
    case PlanningStrategy::ConstrainedOneByOne:
      rrt_ = std::unique_ptr<mrp::VoxbloxOmplRrt>(
          new mrp::VoxbloxOmplRrt(nh_, nh_private_));
      break;
    default:
      ROS_ERROR("[MR Global Planner] Somethig wrong with the setup!");
      rrt_ = std::unique_ptr<mrp::VoxbloxOmplRrt>(
          new mrp::VoxbloxOmplRrt(nh_, nh_private_));
      break;
  }

  // Planner setup
  rrt_->setParameters(params_);
  rrt_->setOptimistic(true);

  rrt_->setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_->setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Interpolator setup
  polynomial_interpolator_ = std::unique_ptr<mrp::PolynomialInterpolator>(
      new mrp::PolynomialInterpolator(dynamic_params_));
  ramp_interpolator_ = std::unique_ptr<mrp::RampInterpolator>(
      new mrp::RampInterpolator(dynamic_params_.v_max, dynamic_params_.a_max,
                                dynamic_params_.sampling_dt));
}

MultiRobotGlobalPlanner::~MultiRobotGlobalPlanner() { planning_timer_.stop(); }

void MultiRobotGlobalPlanner::initROS() {
  // Initialize services, publishers and subscribers. Here we have only
  // one service to trigger the global planner, while the publishers of the
  // global paths are seprated.
  planner_srv_ = nh_private_.advertiseService(
      "plan", &MultiRobotGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &MultiRobotGlobalPlanner::publishPathCallback, this);

  path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "global_path", 1, true);
  waypoints_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("global_waypoints",
                                                             1, true);
  for (uint64_t id = 0; id < num_agents_; ++id) {
    ros::Publisher waypoint_pub =
        nh_.advertise<nav_msgs::Path>("global_path_" + std::to_string(id), 1);
    global_paths_pubs_.push_back(waypoint_pub);

    ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>(
        "odometry_" + std::to_string(id), 10,
        boost::bind(&mrp::MultiRobotGlobalPlanner::odometryCallback, this, _1,
                    id));
    odometry_subs_.push_back(odom_sub);

    ros::Subscriber transf_map_odom =
        nh_.subscribe<geometry_msgs::TransformStamped>(
            "odom_to_map" + std::to_string(id), 10,
            boost::bind(&mrp::MultiRobotGlobalPlanner::odomToMapCallback, this,
                        _1, id));
    transf_odom_map_subs_.push_back(transf_map_odom);

    ros::Subscriber transf_world_map =
        nh_.subscribe<geometry_msgs::TransformStamped>(
            "map_to_world" + std::to_string(id), 10,
            boost::bind(&mrp::MultiRobotGlobalPlanner::mapToWorldCallback, this,
                        _1, id));
    transf_map_world_subs_.push_back(transf_world_map);
  }

  return_home_sub_ = nh_private_.subscribe(
      "return_home", 10, &mrp::MultiRobotGlobalPlanner::returnHomeCallback,
      this);
  move_next_waypoint_sub_ = nh_private_.subscribe(
      "move_next_waypoint", 10,
      &mrp::MultiRobotGlobalPlanner::moveNextWaypointCallback, this);
  replan_to_current_waypoint_sub_ = nh_private_.subscribe(
      "replan_to_current_waypoint", 10,
      &mrp::MultiRobotGlobalPlanner::replanToCurrentWaypointCallback, this);

  // Wait until GPS reference parameters are initialized.
  while (!geodetic_converter_.isInitialised() && coordinate_type_ == "gps") {
    ROS_INFO_ONCE("[MR Global Planner] Waiting for GPS reference parameters..");

    std::vector<double> gps_reference(3, 0.0);  // Latitude, longitude, altitude
    if (nh_private_.getParam("gps_reference", gps_reference)) {
      geodetic_converter_.initialiseReference(
          gps_reference[0], gps_reference[1], gps_reference[2]);
    } else if (nh_private_.getParam("/gps_ref_latitude", gps_reference[0]) &&
               nh_private_.getParam("/gps_ref_longitude", gps_reference[1]) &&
               nh_private_.getParam("/gps_ref_altitude", gps_reference[2])) {
      geodetic_converter_.initialiseReference(
          gps_reference[0], gps_reference[1], gps_reference[2]);
    } else {
      if (params_.verbose_planner) {
        ROS_INFO(
            "[MR Global Planner] GPS reference not ready yet, use  "
            "set_gps_reference_node to set it or initialize in the config "
            "the 'gps_reference' parameter");
      }
      ros::Duration(0.5).sleep();
    }
  }

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner[ ROS initialization complete");
  }

  // Set up ROS timer
  ros::TimerOptions timer_options(
      ros::Duration(timer_dt_),
      boost::bind(&mrp::MultiRobotGlobalPlanner::plannerTimerCallback, this,
                  _1),
      &planning_queue_);
  planning_timer_ = nh_.createTimer(timer_options);
  planning_spinner_.start();

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner] ROS Communication started");
  }
}

bool MultiRobotGlobalPlanner::readParameters() {
  CHECK(nh_private_.getParam("num_agents", num_agents_))
      << "[MR Global Planner] Number of agents not set";

  // Initialize agents' container
  agents_.resize(num_agents_);
  robot_radii_.reserve(num_agents_);

  CHECK(nh_private_.getParam("timer_dt", timer_dt_))
      << "[MR Global Planner] Not specified time for spinner";

  CHECK(nh_private_.getParam("clock_replanning", clock_replanning_))
      << "[MR Global Planner] Not specified clock for replanning";

  CHECK(nh_private_.getParam("world_frame", world_frame_))
      << "[MR Global Planner] World frame name not set";

  CHECK(nh_private_.getParam("map_frame", map_frame_))
      << "[MR Global Planner] Map frame name not set";
  map_frame_ += "_";

  CHECK(nh_private_.getParam("odometry_frame", odometry_frame_))
      << "[MR Global Planner] Odometry frame name not set";
  odometry_frame_ += "_";

  CHECK(nh_private_.getParam("agent_frame", agent_frame_))
      << "[MR Global Planner] Agent frame name not set";
  agent_frame_ += "_";

  // Get coordinate type
  CHECK(nh_private_.getParam("coordinate_type", coordinate_type_))
      << "[MR Global Planner] Coordinate type not specified";
  if (coordinate_type_ != "gps" && coordinate_type_ != "enu") {
    ROS_FATAL(
        "[MR Global Planner] Specified wrong type of coordinates for the"
        " waypoint list!");
    return false;
  }

  // Complete the initialization: read the path planner parameters
  readPlannerParamsFromServer();

  // Initialize ROS
  initROS();

  // Read the waypoint list to reach
  CHECK(readWaypointLists()) << "[MR Global Planner] Could not read waypoint "
                                "list";

  // Initialize the global path storage with empty paths
  for (size_t id = 0; id < num_agents_; ++id) {
    agents_[id].global_path = GlobalPath();
  }

  return true;
}

void MultiRobotGlobalPlanner::readPlannerParamsFromServer() {
  for (int id = 0; id < num_agents_; ++id) {
    double radius(1.0);
    if (!nh_private_.param("robot_radius_" + std::to_string(id), radius,
                           radius)) {
      ROS_WARN("[MR Global Planner] Robot radius not set. Using 1.0 m");
    }
    robot_radii_.push_back(radius);
  }

  params_.planning_height = 1.50;
  if (!nh_private_.param("planning_height", params_.planning_height,
                         params_.planning_height)) {
    ROS_WARN("[MR Global Planner] Planning height not set. Using 1.50 m");
  }

  params_.num_seconds_to_plan = 5.0;
  if (!nh_private_.param("num_seconds_to_plan", params_.num_seconds_to_plan,
                         params_.num_seconds_to_plan)) {
    ROS_WARN(
        "[MR Global Planner] Number of seconds to plan not set. Using 5.0 "
        "s");
  }

  params_.safety_factor = 2.0;
  if (!nh_private_.param("safety_factor", params_.safety_factor,
                         params_.safety_factor)) {
    ROS_WARN(
        "[MR Global Planner] Safety factor for cross-agent checking to "
        "plan not set. Using 2.0");
  }

  params_.simplify_solution = true;
  if (!nh_private_.param("simplify_solution", params_.simplify_solution,
                         params_.simplify_solution)) {
    ROS_WARN("[MR Global Planner] Simplify solution not set. Using true");
  }

  params_.lower_bound = Eigen::Vector3d(-20.0, -20.0, 0.2);
  if (!nh_private_.param("lower_bound_x", params_.lower_bound(0),
                         params_.lower_bound(0))) {
    ROS_WARN("[MR Global Planner] Lower bound X not set. Using -20.0 m");
  }
  if (!nh_private_.param("lower_bound_y", params_.lower_bound(1),
                         params_.lower_bound(1))) {
    ROS_WARN("[MR Global Planner] Lower bound Y not set. Using -20.0 m");
  }
  if (!nh_private_.param("lower_bound_z", params_.lower_bound(2),
                         params_.lower_bound(2))) {
    ROS_WARN("[MR Global Planner] Lower bound Z not set. Using 0.2 m");
  }

  params_.upper_bound = Eigen::Vector3d(20.0, 20.0, 3.0);
  if (!nh_private_.param("upper_bound_x", params_.upper_bound(0),
                         params_.upper_bound(0))) {
    ROS_WARN("[MR Global Planner] Upper bound X not set. Using 20.0 m");
  }
  if (!nh_private_.param("upper_bound_y", params_.upper_bound(1),
                         params_.upper_bound(1))) {
    ROS_WARN("[MR Global Planner] Upper bound Y not set. Using 20.0 m");
  }
  if (!nh_private_.param("upper_bound_z", params_.upper_bound(2),
                         params_.upper_bound(2))) {
    ROS_WARN("[MR Global Planner] Upper bound Z not set. Using 3.0 m");
  }

  params_.bounding_box_inflation = 0.1;
  if (!nh_private_.param("bounding_box_inflation",
                         params_.bounding_box_inflation,
                         params_.bounding_box_inflation)) {
    ROS_WARN("[MR Global Planner] Bounding box inflation not set. Using 0.1 m");
  }

  params_.trust_approx_solution = false;
  if (!nh_private_.param("trust_approx_solution", params_.trust_approx_solution,
                         params_.trust_approx_solution)) {
    ROS_WARN(
        "[MR Global Planner] Whether to trust approximate solution not "
        "set. Using false");
  }

  params_.optimistic = true;
  if (!nh_private_.param("optimistic", params_.optimistic,
                         params_.optimistic)) {
    ROS_WARN(
        "[MR Global Planner] Whether to be optimistic not set. Using "
        "true");
  }

  params_.use_distance_threshold = true;
  if (!nh_private_.param("use_distance_threshold",
                         params_.use_distance_threshold,
                         params_.use_distance_threshold)) {
    ROS_WARN(
        "[MR Global Planner] Whether to use distance threshold not set. "
        "Using true");
  }

  params_.optimization_objective = OptimizationObjective::kDefault;
  int opt_obj;
  if (!nh_private_.param("optimization_objective", opt_obj, opt_obj)) {
    ROS_WARN(
        "[MR Global Planner] Optimization Objective not set. "
        "Using default");
  } else {
    params_.optimization_objective =
        static_cast<OptimizationObjective>(opt_obj);
  }

  switch (params_.optimization_objective) {
    case 0:
      ROS_INFO("[MR Global Planner] Objective: Default");
      break;
    case 1:
      ROS_INFO("[MR Global Planner] Objective: Altitude");
      break;
    default:
      ROS_WARN(
          "[MR Global Planner] Objective not specified. "
          "Using Default");
      params_.optimization_objective = OptimizationObjective::kDefault;
      break;
  }

  if (params_.use_distance_threshold) {
    params_.distance_threshold = 1.10;
    if (!nh_private_.param("distance_threshold", params_.distance_threshold,
                           params_.distance_threshold)) {
      ROS_WARN(
          "[MR Global Planner] Value distance threshold not set. "
          "Using 1.10");
    }
  }

  int planner_type;
  nh_private_.param("planner_type", planner_type, planner_type);
  params_.planner_type = static_cast<RrtPlannerType>(planner_type);

  switch (params_.planner_type) {
    case 0:
      ROS_INFO("[MR Global Planner] Planner: RRT Connect");
      break;
    case 1:
      ROS_INFO("[MR Global Planner] Planner: RRT*");
      break;
    case 2:
      ROS_INFO("[MR Global Planner] Planner: Informed RRT*");
      break;
    case 3:
      ROS_INFO("[MR Global Planner] Planner: BIT*");
      break;
    case 4:
      ROS_INFO("[MR Global Planner] Planner: PRM");
      break;
    default:
      ROS_WARN(
          "[MR Global Planner] Global Planner solver not specified. "
          "Using RRT*");
      params_.planner_type = RrtPlannerType::kRrtStar;
      break;
  }

  params_.goal_threshold = 0.2;
  if (!nh_private_.param("goal_threshold", params_.goal_threshold,
                         params_.goal_threshold)) {
    ROS_WARN("[MR Global Planner] Goal reached threshold not set. Using 0.2 m");
  }

  params_.altitude_obj_params.alpha = 1.0;
  if (!nh_private_.param("altitude_obj_params/alpha",
                         params_.altitude_obj_params.alpha,
                         params_.altitude_obj_params.alpha)) {
    ROS_WARN("[MR Global Planner] Altitude objective alpha not set. Using 1.0");
  }

  params_.altitude_obj_params.beta = 10.0;
  if (!nh_private_.param("altitude_obj_params/beta",
                         params_.altitude_obj_params.beta,
                         params_.altitude_obj_params.beta)) {
    ROS_WARN("[MR Global Planner] Altitude objective beta not set. Using 10.0");
  }

  int planning_strategy;
  nh_private_.param("planning_strategy", planning_strategy, planning_strategy);
  params_.planning_strategy = static_cast<PlanningStrategy>(planning_strategy);

  switch (params_.planning_strategy) {
    case PlanningStrategy::OneByOne:
      ROS_INFO("[MR Global Planner] Planning Strategy: One By One");
      break;
    case PlanningStrategy::OneShot:
      ROS_INFO("[MR Global Planner] Planning Strategy: One Shot");
      break;
    case PlanningStrategy::ConstrainedOneByOne:
      ROS_INFO("[MR Global Planner] Planning Strategy: Constrained One By One");
      break;
    default:
      ROS_WARN(
          "[MR Global Planner] Planning Strategy not specified. "
          "Using Constrained One By One");
      params_.planning_strategy = PlanningStrategy::ConstrainedOneByOne;
      break;
  }

  int interpolator;
  nh_private_.param("interpolator", interpolator, interpolator);
  params_.interpolator = static_cast<Interpolator>(interpolator);

  switch (params_.interpolator) {
    case Interpolator::Polynomial:
      ROS_INFO("[MR Global Planner] Interpolator: Polynomial");
      break;
    case Interpolator::Ramp:
      ROS_INFO("[MR Global Planner] Interpolator: Ramp");
      break;
    default:
      ROS_WARN(
          "[MR Global Planner] Planning Strategy not specified. "
          "Using Constrained One By One");
      params_.interpolator = Interpolator::Polynomial;
      break;
  }

  params_.visualize = true;
  if (!nh_private_.param("visualize", params_.visualize, params_.visualize)) {
    ROS_WARN("[MR Global Planner] Visualization not set. Using true");
  }

  params_.scale_factor_visualization = 1.0;
  if (!nh_private_.param("scale_factor_visualization",
                         params_.scale_factor_visualization,
                         params_.scale_factor_visualization)) {
    ROS_WARN(
        "[MR Global Planner] Scale factor for visualization not set. "
        "Using 1.0");
  }

  params_.verbose_planner = false;
  if (!nh_private_.param("verbose_planner", params_.verbose_planner,
                         params_.verbose_planner)) {
    ROS_WARN("[MR Global Planner] Verbosity not set. Using false");
  }

  // Dynamic parameters
  dynamic_params_.v_max = 1.0;
  if (!nh_private_.param("v_max", dynamic_params_.v_max,
                         dynamic_params_.v_max)) {
    ROS_WARN("[MR Global Planner] Velocity max not set. Using 1.0 m/s");
  }

  dynamic_params_.a_max = 1.0;
  if (!nh_private_.param("a_max", dynamic_params_.a_max,
                         dynamic_params_.a_max)) {
    ROS_WARN("[MR Global Planner] Acceleration max not set. Using 1.0 m/s2");
  }

  dynamic_params_.v_yaw_max = 1.0;
  if (!nh_private_.param("v_yaw_max", dynamic_params_.v_yaw_max,
                         dynamic_params_.v_yaw_max)) {
    ROS_WARN("[MR Global Planner] Velocity max yaw not set. Using 1.0 rad/s");
  }

  dynamic_params_.a_yaw_max = 1.0;
  if (!nh_private_.param("a_yaw_max", dynamic_params_.a_yaw_max,
                         dynamic_params_.a_yaw_max)) {
    ROS_WARN(
        "[MR Global Planner] Acceleration max yaw not set. Using 1.0 "
        "rad/s2");
  }

  dynamic_params_.sampling_dt = 0.05;
  if (!nh_private_.param("sampling_dt", dynamic_params_.sampling_dt,
                         dynamic_params_.sampling_dt)) {
    ROS_WARN("[MR Global Planner] Sampling time not set. Using 0.05 s");
  }
}

bool MultiRobotGlobalPlanner::readWaypointLists() {
  // Clear old list
  waypoints_lists_.clear();

  // Iterate over the agents
  for (int id = 0; id < num_agents_; ++id) {
    // Fetch the trajectory from the parameter server.
    std::vector<double> easting;
    std::vector<double> northing;
    std::vector<double> height;
    WaypointsList waypoints;

    std::string agent_name = "agent_" + std::to_string(id);
    CHECK(nh_private_.getParam(agent_name + "/easting", easting) &&
          nh_private_.getParam(agent_name + "/northing", northing) &&
          nh_private_.getParam(agent_name + "/height", height))
        << "Error loading path parameters!";

    // Check for valid trajectory inputs.
    bool valid_sizes =
        easting.size() == northing.size() && northing.size() == height.size();
    CHECK(valid_sizes) << "Error: path parameter arrays are not the same size";

    // Add (x,y,z) co-ordinates from file to path.
    for (size_t i = 0; i < easting.size(); i++) {
      Eigen::Vector3d cwp;
      // GPS path co-ordinates.
      if (coordinate_type_ == "gps") {
        double initial_latitude;
        double initial_longitude;
        double initial_altitude;

        // Convert GPS point to ENU co-ordinates.
        // NB: waypoint altitude = desired height above reference + registered
        // reference altitude.
        geodetic_converter_.getReference(&initial_latitude, &initial_longitude,
                                         &initial_altitude);
        geodetic_converter_.geodetic2Enu(northing[i], easting[i],
                                         (initial_altitude + height[i]),
                                         &cwp.x(), &cwp.y(), &cwp.z());
      }
      // ENU path co-ordinates.
      else if (coordinate_type_ == "enu") {
        cwp.x() = easting[i];
        cwp.y() = northing[i];
        cwp.z() = height[i];
      }
      waypoints.push_back(cwp);
    }

    // Add the waypoint list to the storage
    waypoints_lists_.push_back(waypoints);

    if (params_.verbose_planner) {
      ROS_INFO_STREAM(
          "[MR Global Planner] Path loaded from file. Number of points "
          "in path for agent "
          << id << ": " << waypoints.size());
    }
  }

  // Publish the visualization
  if (params_.visualize) {
    publishMarkerWaypoints();
  }
  return true;
}

bool MultiRobotGlobalPlanner::plannerServiceCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // Check that the frames exits first
  bool frames_exists = tf_listener_.frameExists(world_frame_);
  for (int id = 0; id < num_agents_; ++id) {
    frames_exists &=
        tf_listener_.frameExists(odometry_frame_ + std::to_string(id));
    frames_exists &=
        tf_listener_.frameExists(agent_frame_ + std::to_string(id));

    // Get the positions of all the agents to set the home position
    if (frames_exists) {
      Eigen::Vector4d pose_id;
      getPoseAgentId(id, pose_id);
      agents_[id].home_position = pose_id.head(3);
    } else {
      if (params_.verbose_planner) {
        ROS_WARN_STREAM(
            "[MR Global Planner] Not all frames for "
            "agent "
            << id << " exist");
      }
      break;
    }
  }

  // Check if we have initialized the transformations
  bool valid_transformations = true;
  for (int64_t id = 0; id < num_agents_; ++id) {
    valid_transformations &= agents_[id].transformations_initialized_O_A;
    valid_transformations &= agents_[id].transformations_initialized_M_O;
    valid_transformations &= agents_[id].transformations_initialized_W_M;

    if (!valid_transformations) {
      if (params_.verbose_planner) {
        ROS_WARN_STREAM(
            "[MR Global Planner] Not all transformations for "
            "agent "
            << id << " have been initialized");
      }
      break;
    }
  }

  if (!frames_exists && !valid_transformations) {
    ROS_ERROR("[MR Global Planner] The TF tree is not complete, cannot plan");
    res.success = false;
    return false;
  }

  // Check if we have a global map
  Eigen::Vector3d lower_bound, upper_bound;
  computeMapBounds(&lower_bound, &upper_bound);
  if ((upper_bound.x() - lower_bound.x()) < 0.0 ||
      (upper_bound.y() - lower_bound.y()) < 0.0 ||
      (upper_bound.z() - lower_bound.z()) < 0.0) {
    ROS_ERROR("[MR Global Planner] The map is empty, cannot plan");
    res.success = false;
    return false;
  }

  // Everything is fine here
  perform_planning_ = true;
  res.success = true;

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner] Triggered global planner");
  }

  return true;
}

void MultiRobotGlobalPlanner::odometryCallback(
    const nav_msgs::OdometryConstPtr &odom_msg, const uint64_t agent_id) {
  // Extract the info in the message
  Eigen::Vector3d position(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
  Eigen::Quaterniond orientation;
  tf::quaternionMsgToEigen(odom_msg->pose.pose.orientation, orientation);

  // Store in the rigth transformation storage
  Eigen::Matrix4d T_O_A_id;
  T_O_A_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientation);
  T_O_A_id.block<3, 1>(0, 3) = position;
  T_O_A_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  agents_[agent_id].T_O_A = T_O_A_id;

  // Update flag
  agents_[agent_id].transformations_initialized_O_A = true;
  ROS_INFO_STREAM_ONCE(
      "[MR Global Planner] Got first odometry message for "
      "agent "
      << agent_id);
}

void MultiRobotGlobalPlanner::odomToMapCallback(
    const geometry_msgs::TransformStampedConstPtr transf_odom_map_msg,
    const uint64_t agent_id) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientat;
  tf::vectorMsgToEigen(transf_odom_map_msg->transform.translation, position);
  tf::quaternionMsgToEigen(transf_odom_map_msg->transform.rotation, orientat);

  Eigen::Matrix4d T_M_O_id;
  T_M_O_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientat);
  T_M_O_id.block<3, 1>(0, 3) = position;
  T_M_O_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  agents_[agent_id].T_M_O = T_M_O_id;

  // Update flag
  agents_[agent_id].transformations_initialized_M_O = true;
  ROS_INFO_STREAM_ONCE(
      "[MR Global Planner] Got first odom to map "
      "transformation for agent "
      << agent_id);
}

void MultiRobotGlobalPlanner::mapToWorldCallback(
    const geometry_msgs::TransformStampedConstPtr transf_map_world_msg,
    const uint64_t agent_id) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientat;
  tf::vectorMsgToEigen(transf_map_world_msg->transform.translation, position);
  tf::quaternionMsgToEigen(transf_map_world_msg->transform.rotation, orientat);

  Eigen::Matrix4d T_W_M_id;
  T_W_M_id.block<3, 3>(0, 0) = Eigen::Matrix3d(orientat);
  T_W_M_id.block<3, 1>(0, 3) = position;
  T_W_M_id.block<1, 4>(3, 0) << 0.0, 0.0, 0.0, 1.0;
  agents_[agent_id].T_W_M = T_W_M_id;

  // Update flag
  agents_[agent_id].transformations_initialized_W_M = true;
  ROS_INFO_STREAM_ONCE(
      "[MR Global Planner] Got first map to odom "
      "transformation for agent "
      << agent_id);
}

void MultiRobotGlobalPlanner::returnHomeCallback(
    const std_msgs::Int16ConstPtr &home_msg) {
  // Check condition
  if (params_.planning_strategy == PlanningStrategy::OneShot) {
    ROS_ERROR(
        "[MR Global Planner] For the 'One Shot' strategy no return home"
        " behaviour is implemented");
    return;
  }

  // Get the id
  int16_t id = home_msg->data;

  // Check condition to send all the agents back home
  if (id == -1) {
    if (params_.verbose_planner) {
      ROS_INFO(
          "[MR Global Planner] Triggered return home behaviour for "
          "all the agents");
    }
    for (int16_t i = 0; i < num_agents_; ++i) {
      agents_[i].home_triggered = true;
      agents_[i].valid_path = false;
      sendStopCommand(i);
    }
    return;
  }

  // Else: process the current id
  if (params_.verbose_planner) {
    ROS_INFO_STREAM(
        "[MR Global Planner] Triggered return home behaviour for "
        "agent "
        << id);
  }

  // Send stop command
  agents_[id].home_triggered = true;
  agents_[id].valid_path = false;
  sendStopCommand(id);

  // In case we use the hierarchical planner, re-adjust the prioritity queue.
  // This means: if we trigger the return home behaviour for one drone, all
  // the drone that are lower in the hierarchy should replan their path
  // if(params_.planning_strategy == PlanningStrategy::ConstrainedOneByOne) {
  //  for (int16_t i = id + 1; i < num_agents_; ++i) {
  //    agents_[i].valid_path = false;
  //    sendStopCommand(i);
  //  }
  //}
}

void MultiRobotGlobalPlanner::moveNextWaypointCallback(
    const std_msgs::Int16ConstPtr &home_msg) {
  int64_t id = home_msg->data;
  uint64_t index_wp = agents_[id].waypoint_num;

  if (params_.planning_strategy == PlanningStrategy::OneShot) {
    ROS_ERROR(
        "[MR Global Planner] Move to next waypont behaviour not "
        "implemented for OneShot strategy");
    return;
  }

  if (index_wp + 1 < waypoints_lists_[id].size()) {
    agents_[id].waypoint_num++;
    agents_[id].move_next_waypoint = true;

    if (params_.verbose_planner) {
      ROS_INFO_STREAM("[MR Global Planner] Agent " << id
                                                   << " moving to next "
                                                      "waypoint");
    }
  } else {
    agents_[id].move_next_waypoint = false;

    if (params_.verbose_planner) {
      ROS_WARN_STREAM("[MR Global Planner] Agent "
                      << id
                      << " does not have "
                         "any more waypoints to reach");
    }
  }
}

void MultiRobotGlobalPlanner::replanToCurrentWaypointCallback(
    const std_msgs::Int16ConstPtr &replan_msg) {
  int64_t id = replan_msg->data;
  if (params_.verbose_planner) {
    ROS_INFO_STREAM(
        "[MR Global Planner] Replanning to current waypoint for "
        "agent "
        << id);
  }
  agents_[id].valid_path = false;
}

bool MultiRobotGlobalPlanner::publishPathCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  return publishPathsToAgents();
}

void MultiRobotGlobalPlanner::plannerTimerCallback(
    const ros::TimerEvent &event) {
  // Check if we have to perform planning at all
  if (!perform_planning_) {
    return;
  }

  if (params_.planning_strategy == PlanningStrategy::OneByOne) {
    planOneByOne();
  } else if (params_.planning_strategy == PlanningStrategy::OneShot) {
    planOneShot();
  } else if (params_.planning_strategy ==
             PlanningStrategy::ConstrainedOneByOne) {
    planConstrainedOneByOne();
  }

  // Update iteration counter
  ++iter_timer_;

  // Publish visualization
  if (params_.visualize) {
    publishMarkerPaths();
  }
}

void MultiRobotGlobalPlanner::planOneByOne() {
  // Ok, we need to plan. So iterate over the agents and check what's going
  // on. If an agent is in collision, then plan to either the current or the
  // next waypoint. If the waypoint has been reached, then move to the next
  // on in the list.

  // Check if we have to force the replanning
  bool force_replanning = checkForceReplanning();

  for (size_t id = 0; id < num_agents_; ++id) {
    Eigen::Vector4d agent_pose;
    if (!getPoseAgentId(id, agent_pose)) {
      ROS_ERROR_STREAM("[MR Global Planner] Could not retrieve pose of agent "
                       << id);
      return;
    }

    // Check if we have to plan home
    if (isAgentSentHome(id)) {
      planToHomeAgent(agent_pose, id);
      continue;
    }

    // Extract the waypoint to plan to
    uint64_t index_wp = agents_[id].waypoint_num;

    // Start the checks here
    bool goal_reached =
        isGoalReachedAgent(id, agent_pose) || agents_[id].reached_all_goals;
    bool goal_reacheable =
        checkOptimisticMapCollision(waypoints_lists_[id][index_wp], id);
    bool path_in_collision = checkPathsForCollisions(agent_pose, id);
    bool move_next_waypoint = agents_[id].move_next_waypoint;

    if (!goal_reacheable && params_.verbose_planner) {
      ROS_WARN_STREAM("[MR Global Planner] Current goal for agent "
                      << id << " is not reacheable");
    }

    if (!goal_reached && !path_in_collision && goal_reacheable &&
        !force_replanning && !move_next_waypoint) {
      // If we have a valid path and the goal has not been reached yet, move to
      // next agent
      continue;
    } else if ((goal_reached || !goal_reacheable) && !move_next_waypoint) {
      // The goal has been reached or is in invalid position. Move to next
      // waypoint and plan
      if (index_wp + 1 < waypoints_lists_[id].size()) {
        agents_[id].waypoint_num++;
      } else {
        if (params_.verbose_planner) {
          ROS_INFO_STREAM("[MR Global Planner] Agent "
                          << id << " has reached all the waypoints");
        }
        agents_[id].reached_all_goals = true;

        // Inform the local planner
        callStopSrvLocalPlanner(id);
        continue;
      }
      computePathToWaypoint(agent_pose, id);
    } else if (path_in_collision || force_replanning || move_next_waypoint) {
      // Send the stop command if we have an older path
      if (!agents_[id].global_path.empty()) {
        sendStopCommand(id);
      }

      // Make sure we don't keep trying reaching next waypoint at every timer
      // iteration
      agents_[id].move_next_waypoint = false;

      // If the path is not valid, then move to the next waypoint if the
      // current goal is not valid.
      if (goal_reacheable) {
        computePathToWaypoint(agent_pose, id);
      } else {  // ie current goal is not valid
        if (index_wp + 1 < waypoints_lists_[id].size()) {
          agents_[id].waypoint_num++;
        } else {
          if (params_.verbose_planner) {
            ROS_INFO_STREAM("[MR Global Planner] Agent "
                            << id << " has reached all the waypoints");
          }
          agents_[id].reached_all_goals = true;

          // Inform the local planner
          callStopSrvLocalPlanner(id);
          continue;
        }
        computePathToWaypoint(agent_pose, id);
      }
    }
  }
}

void MultiRobotGlobalPlanner::planConstrainedOneByOne() {
  for (size_t id = 0; id < num_agents_; ++id) {
    Eigen::Vector4d agent_pose;
    if (!getPoseAgentId(id, agent_pose)) {
      ROS_ERROR_STREAM("[MR Global Planner] Could not retrieve pose of agent "
                       << id);
      return;
    }

    // Check if we have to force the replanning
    bool force_replanning = checkForceReplanning();

    // Check if we have to plan home
    if (isAgentSentHome(id)) {
      planToHomeConstrainedAgent(agent_pose, id);
      continue;
    }

    // Start the checks here
    uint64_t index_wp = agents_[id].waypoint_num;
    bool goal_reached =
        isGoalReachedAgent(id, agent_pose) || agents_[id].reached_all_goals;
    bool goal_reacheable =
        checkOptimisticMapCollision(waypoints_lists_[id][index_wp], id);
    bool path_in_collision = checkPathsForCollisions(agent_pose, id);
    bool move_next_waypoint = agents_[id].move_next_waypoint;

    if (!goal_reacheable && params_.verbose_planner) {
      ROS_WARN_STREAM("[MR Global Planner] Current goal for agent "
                      << id << " is not reacheable");
    }

    if (!goal_reached && !path_in_collision && goal_reacheable &&
        !force_replanning && !move_next_waypoint) {
      // If we have a valid path and the goal has not been reached yet, move to
      // next agent
      continue;
    } else if ((goal_reached || !goal_reacheable) && !move_next_waypoint) {
      // The goal has been reached or is in invalid position. Move to next
      // waypoint and plan
      if (index_wp + 1 < waypoints_lists_[id].size()) {
        agents_[id].waypoint_num++;
      } else {
        if (params_.verbose_planner) {
          ROS_INFO_STREAM("[MR Global Planner] Agent "
                          << id << " has reached all the waypoints");
        }
        agents_[id].reached_all_goals = true;

        // Inform the local planner
        callStopSrvLocalPlanner(id);
        continue;
      }
      computeConstrainedPathToWaypoint(agent_pose, id);
    } else if (path_in_collision || force_replanning || move_next_waypoint) {
      // Send the stop command if we have an older path
      if (!agents_[id].global_path.empty()) {
        sendStopCommand(id);
      }

      // Make sure we don't keep trying reaching next waypoint at every timer
      // iteration
      agents_[id].move_next_waypoint = false;

      // If the path is not valid, then move to the next waypoint if the
      // current goal is not valid.
      if (goal_reacheable) {
        computeConstrainedPathToWaypoint(agent_pose, id);
      } else {  // ie current goal is not valid
        if (index_wp + 1 < waypoints_lists_[id].size()) {
          agents_[id].waypoint_num++;
        } else {
          if (params_.verbose_planner) {
            ROS_INFO_STREAM("[MR Global Planner] Agent "
                            << id << " has reached all the waypoints");
          }
          agents_[id].reached_all_goals = true;

          // Inform the local planner
          callStopSrvLocalPlanner(id);
          continue;
        }
        computeConstrainedPathToWaypoint(agent_pose, id);
      }
    }

    // Re-adjust the hierarchy: if we replanned for this agent, we need to
    // replan for all the other agents that are below in hierarchy
    // for(size_t i = id + 1; i < num_agents_; ++i) {
    //  agents_[i].valid_path = false;
    //  sendStopCommand(i);
    //}
  }
}

void MultiRobotGlobalPlanner::planOneShot() {
  // Extract all current global goals. If one agent has reached its current
  // goal, move to the next one in the list.
  mav_msgs::EigenTrajectoryPointVector starts, goals;
  bool path_in_collision = true;
  bool all_agents_reached_goals = true;
  bool need_to_plan = checkForceReplanning();

  for (size_t id = 0; id < num_agents_; ++id) {
    // Start
    Eigen::Vector4d agent_pose;
    if (!getPoseAgentId(id, agent_pose)) {
      ROS_ERROR_STREAM("[MR Global Planner] Could not retrieve pose of agent "
                       << id);
      return;
    }
    mav_msgs::EigenTrajectoryPoint start;
    start.position_W = agent_pose.head(3);
    starts.push_back(start);

    // Goal
    uint64_t index_wp = agents_[id].waypoint_num;
    mav_msgs::EigenTrajectoryPoint goal;

    bool goal_reached =
        isGoalReachedAgent(id, agent_pose) || agents_[id].reached_all_goals;
    bool goal_reacheable =
        checkOptimisticMapCollision(waypoints_lists_[id][index_wp], id);

    if (params_.verbose_planner && !goal_reacheable) {
      ROS_INFO_STREAM("[MR Global Planner] Next goal for agent "
                      << id << " is not reacheable!");
    }

    if (goal_reached || !goal_reacheable) {
      while (goal_reached || !goal_reacheable) {
        if (index_wp + 1 < waypoints_lists_[id].size()) {
          agents_[id].waypoint_num++;
          index_wp = agents_[id].waypoint_num;
        } else {
          if (params_.verbose_planner) {
            ROS_INFO_STREAM("[MR Global Planner] Agent "
                            << id << " has finished the waypoints list");
          }
          agents_[id].reached_all_goals = true;

          // In this case, set the goal as the start state
          goals.push_back(start);
          break;
        }

        goal_reacheable =
            checkOptimisticMapCollision(waypoints_lists_[id][index_wp], id);
        goal_reached =
            isGoalReachedAgent(id, agent_pose) || agents_[id].reached_all_goals;
      }
      need_to_plan = true;
    } else {
      need_to_plan = false;
    }
    goal.position_W = waypoints_lists_[id][index_wp];
    goals.push_back(goal);

    // Check if any of the paths is in collision
    path_in_collision &= checkPathsForCollisions(agent_pose, id);

    // Check if we have to plan or if we have reached all the goals
    all_agents_reached_goals &= goal_reached && agents_[id].reached_all_goals;

    // Now decide if we need to plan
    need_to_plan |= path_in_collision;
  }

  if (!need_to_plan || all_agents_reached_goals) {
    // If there are no collisions, then we can just return
    return;
  }

  // If one of the paths is in collisions, then we replan for everybody
  Eigen::Vector3d inflation(params_.bounding_box_inflation,
                            params_.bounding_box_inflation, 0.0);
  // Don't in flate in z. ;)
  rrt_->setBounds(params_.lower_bound - inflation,
                  params_.upper_bound + inflation);
  auto max_radius =
      std::max_element(std::begin(robot_radii_), std::end(robot_radii_));
  rrt_->setRobotRadius(*max_radius);
  rrt_->setupMultiAgentProblem(num_agents_);

  std::vector<mav_msgs::EigenTrajectoryPoint::Vector> paths;
  if (!rrt_->getBestPathTowardGoalMultiAgent(starts, goals, paths)) {
    if (params_.verbose_planner) {
      ROS_ERROR("[MR Global Planner] Planner could not find a solution");
    }
    return;
  }

  for (int id = 0; id < num_agents_; ++id) {
    GlobalPath global_path;
    Eigen::Vector4d agent_pose;
    getPoseAgentId(id, agent_pose);
    global_path.push_back(
        Eigen::Vector4d(paths[id][0].position_W(0), paths[id][0].position_W(1),
                        paths[id][0].position_W(2), agent_pose(3)));

    for (size_t i = 1; i < paths[id].size() - 1; ++i) {
      mav_msgs::EigenTrajectoryPoint point_curr = paths[id][i];
      mav_msgs::EigenTrajectoryPoint point_next = paths[id][i + 1];
      Eigen::Vector3d direction =
          (point_next.position_W - point_curr.position_W).normalized();

      global_path.push_back(Eigen::Vector4d(
          paths[id][i].position_W(0), paths[id][i].position_W(1),
          paths[id][i].position_W(2), std::atan2(direction(1), direction(0))));
    }
    double final_yaw = global_path.back()(3);
    global_path.push_back(Eigen::Vector4d(
        paths[id].back().position_W(0), paths[id].back().position_W(1),
        paths[id].back().position_W(2), final_yaw));

    // Interpolate global path
    GlobalPath interpolated_global_path;
    if (params_.interpolator == Interpolator::Polynomial) {
      polynomial_interpolator_->interpolate(global_path,
                                            &interpolated_global_path);
    } else if (params_.interpolator == Interpolator::Ramp) {
      ramp_interpolator_->interpolate(global_path, &interpolated_global_path);
    }

    agents_[id].global_path = interpolated_global_path;
    agents_[id].valid_path = true;
  }

  // Now publish the paths to all agents at one
  publishPathsToAgents();
}

void MultiRobotGlobalPlanner::planToHomeAgent(const Eigen::Vector4d &agent_pose,
                                              const int agent_id) {
  // Check if we have reached the home position
  if ((agent_pose.head(3) - agents_[agent_id].home_position).norm() <
      params_.goal_threshold) {
    ROS_INFO_STREAM_ONCE("[MR Global Planner] Agent "
                         << agent_id << " has reached its home");
    return;
  }

  // Check if we have to plan
  if (!checkPathsToHomeForCollisions(agent_pose, agent_id)) {
    return;
  }

  // Send stop command in this cas, because the path is in collision
  sendStopCommand(agent_id);

  // Get the position of the agent in the world frame. This is the start
  // position for planning
  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Planning for agent " << agent_id);
  }
  Eigen::Vector3d start = agent_pose.head(3);

  // Get goal position for planning
  Eigen::Vector3d goal = agents_[agent_id].home_position;

  // Directly plan in OMPL using the ESDF (ie already known explored space)
  mav_msgs::EigenTrajectoryPoint start_point, goal_point;
  start_point.position_W = start;
  goal_point.position_W = goal;

  // Get the map bounds to focus the planning in the explored space
  Eigen::Vector3d lower_bound, upper_bound;
  computeMapBounds(&lower_bound, &upper_bound);

  // Inflate the bounds a bit.
  Eigen::Vector3d inflation(params_.bounding_box_inflation,
                            params_.bounding_box_inflation, 0.0);
  // In this case give a very small radius to avoid problems with not properly
  // seen space
  rrt_->setRobotRadius(0.05);
  rrt_->setBounds(lower_bound - inflation, upper_bound + inflation);
  rrt_->setupReturnHomeProblem(start, goal);

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner] Successfully set up problem");
    ROS_INFO_STREAM("Start: [" << start.x() << ", " << start.y() << ", "
                               << start.z() << "]");
    ROS_INFO_STREAM("Goal: [" << goal.x() << ", " << goal.y() << ", "
                              << goal.z() << "]");
  }

  mav_msgs::EigenTrajectoryPoint::Vector path_vector_eigen;
  if (!rrt_->getPathBetweenWaypoints(start_point, goal_point,
                                     &path_vector_eigen)) {
    ROS_ERROR_STREAM(
        "[MR Global Planner] Could not find home path with OMPL "
        "for agent "
        << agent_id);
    agents_[agent_id].valid_path = false;
    return;
  }

  // Store the path from OMPL
  path_vector_eigen[0].setFromYaw(agent_pose(3));
  GlobalPath global_path = getGlobalPathFromEigenVector(path_vector_eigen);

  // Interpolate global path
  GlobalPath interpolated_global_path;
  if (params_.interpolator == Interpolator::Polynomial) {
    polynomial_interpolator_->interpolate(global_path,
                                          &interpolated_global_path);
  } else if (params_.interpolator == Interpolator::Ramp) {
    ramp_interpolator_->interpolate(global_path, &interpolated_global_path);
  }

  agents_[agent_id].global_path = interpolated_global_path;
  agents_[agent_id].valid_path = true;

  publishPathSingleAgents(agent_id);
}

void MultiRobotGlobalPlanner::planToHomeConstrainedAgent(
    const Eigen::Vector4d &agent_pose, const int agent_id) {
  // Check: if we are planning for the first agent (id = 0), then we plan as
  // usual - without any constraints
  if (agent_id == 0) {
    planToHomeAgent(agent_pose, agent_id);
    return;
  }

  // Check if we have reached the home position
  if ((agent_pose.head(3) - agents_[agent_id].home_position).norm() <
      params_.goal_threshold) {
    ROS_INFO_STREAM_ONCE("[MR Global Planner] Agent " << agent_id
                                                      << " has "
                                                         "reached its home");
    return;
  }

  // Check if we have to plan
  if (!checkPathsToHomeForCollisions(agent_pose, agent_id)) {
    return;
  } else if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Agent " << agent_id
                                                 << " in collision");
  }

  // Send stop command in this cas, because the path is in collision
  sendStopCommand(agent_id);

  // Else: we are planning for the other (slave) agents
  // Get the position of the agent in the world frame. This is the start
  // position for planning
  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Planning for agent " << agent_id);
  }
  Eigen::Vector3d start = agent_pose.head(3);

  // Get goal position for planning
  Eigen::Vector3d goal = agents_[agent_id].home_position;

  // Get all the path that constraint the movements
  std::vector<std::vector<Eigen::Vector3d> > constrain_paths(agent_id);
  for (int id = 0; id < agent_id; ++id) {
    for (GlobalPath::iterator it = agents_[id].global_path.begin();
         it < agents_[id].global_path.end(); ++it) {
      constrain_paths[id].push_back((*it).head(3));
    }
  }

  // Directly plan in OMPL using the ESDF (ie already known explored space)
  mav_msgs::EigenTrajectoryPoint start_point, goal_point;
  start_point.position_W = start;
  goal_point.position_W = goal;

  // Get the map bounds to focus the planning in the explored space
  Eigen::Vector3d lower_bound, upper_bound;
  computeMapBounds(&lower_bound, &upper_bound);

  // Inflate the bounds a bit.
  Eigen::Vector3d inflation(params_.bounding_box_inflation,
                            params_.bounding_box_inflation, 0.0);
  rrt_->setRobotRadius(robot_radii_[agent_id]);
  rrt_->setBounds(lower_bound - inflation, upper_bound + inflation);
  rrt_->setConstraintPaths(constrain_paths);
  rrt_->setupReturnHomeProblem(start, goal, true);

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner] Successfully set up problem");
    ROS_INFO_STREAM("Start: [" << start.x() << ", " << start.y() << ", "
                               << start.z() << "]");
    ROS_INFO_STREAM("Goal: [" << goal.x() << ", " << goal.y() << ", "
                              << goal.z() << "]");
  }

  mav_msgs::EigenTrajectoryPoint::Vector path_vector_eigen;
  if (!rrt_->getPathBetweenWaypoints(start_point, goal_point,
                                     &path_vector_eigen)) {
    ROS_ERROR_STREAM(
        "[MR Global Planner] Could not find home path with OMPL "
        "for agent "
        << agent_id);
    agents_[agent_id].valid_path = false;
    return;
  }

  // Store the path from OMPL
  path_vector_eigen[0].setFromYaw(agent_pose(3));
  GlobalPath global_path = getGlobalPathFromEigenVector(path_vector_eigen);

  // Interpolate global path
  GlobalPath interpolated_global_path;
  if (params_.interpolator == Interpolator::Polynomial) {
    polynomial_interpolator_->interpolate(global_path,
                                          &interpolated_global_path);
  } else if (params_.interpolator == Interpolator::Ramp) {
    ramp_interpolator_->interpolate(global_path, &interpolated_global_path);
  }

  agents_[agent_id].global_path = interpolated_global_path;
  agents_[agent_id].valid_path = true;

  publishPathSingleAgents(agent_id);
}

bool MultiRobotGlobalPlanner::checkPathsForCollisions(
    const Eigen::Vector4d &agent_pose, const int agent_id, bool optimistic) {
  // If the agent does not have a global path, then return that we are in
  // collision to enforce the planning
  // FIXME Bad hack!
  if (!agents_[agent_id].valid_path) {
    return true;
  }

  // Get the starting position and find where we are along global path
  Eigen::Vector3d start_position = agent_pose.head(3);
  GlobalPath path = agents_[agent_id].global_path;

  size_t initial_index = 0;
  double dist = (start_position - path[initial_index].head(3)).norm();

  for (size_t i = 1; i < path.size(); ++i) {
    double dist_curr = (start_position - path[i].head(3)).norm();

    if (dist_curr < dist) {
      dist = dist_curr;
      initial_index = i;
    } else {
      break;
    }
  }

  for (size_t i = initial_index; i < path.size(); ++i) {
    if (optimistic) {
      if (!checkOptimisticMapCollision(path[i].head(3), agent_id)) {
        // Collision
        if (params_.verbose_planner) {
          ROS_INFO_STREAM("[MR Global Planner] Agent "
                          << agent_id << " in collision (TSDF)");
        }
        return true;
      }
    } else {
      double distance = getMapDistance(path[i].head(3));
      if (distance < robot_radii_[agent_id]) {
        // Not in free space
        if (params_.verbose_planner) {
          ROS_INFO_STREAM("[MR Global Planner] Agent "
                          << agent_id << " in collision (ESDF) with distance "
                          << distance);
        }
        return true;
      }
    }

    // Check if the current path is in collision with the paths of the other
    // agents (only if we are using 'ConstrainedOneByOne' planner
    if (agent_id > 0 &&
        params_.planning_strategy == PlanningStrategy::ConstrainedOneByOne) {
      Eigen::Vector3d query_position(path[i].head(3));
      for (int id = 0; id < agent_id; ++id) {
        for (GlobalPath::const_iterator it = agents_[id].global_path.begin();
             it < agents_[id].global_path.end(); ++it) {
          if (((*it).head(3) - query_position).norm() <
              params_.safety_factor * robot_radii_[id]) {
            if (params_.verbose_planner) {
              ROS_INFO_STREAM("[MR Global Planner] Agent "
                              << agent_id << " in collision with another path");
            }
            return true;
          }
        }
      }
    }
  }

  // Collision free
  return false;
}

bool MultiRobotGlobalPlanner::checkPathsToHomeForCollisions(
    const Eigen::Vector4d &agent_pose, const int agent_id) const {
  // If the agent does not have a global path, then return that we are in
  // collision to enforce the planning
  // FIXME Bad hack!
  if (!agents_[agent_id].valid_path) {
    return true;
  }

  // Get the starting position and find where we are along global path
  Eigen::Vector3d start_position = agent_pose.head(3);
  GlobalPath path = agents_[agent_id].global_path;

  size_t initial_index = 0;
  double dist = (start_position - path[initial_index].head(3)).norm();

  for (size_t i = 1; i < path.size(); ++i) {
    double dist_curr = (start_position - path[i].head(3)).norm();

    if (dist_curr < dist) {
      dist = dist_curr;
      initial_index = i;
    } else {
      break;
    }
  }

  for (size_t i = initial_index; i < path.size(); ++i) {
    // Check if the current path is in collision with the paths of the other
    // agents (only if we are using 'ConstrainedOneByOne' planner
    if (agent_id > 0 &&
        params_.planning_strategy == PlanningStrategy::ConstrainedOneByOne) {
      Eigen::Vector3d query_position(path[i].head(3));
      for (int id = 0; id < agent_id; ++id) {
        for (GlobalPath::const_iterator it = agents_[id].global_path.begin();
             it < agents_[id].global_path.end(); ++it) {
          if (((*it).head(3) - query_position).norm() <
              params_.safety_factor * robot_radii_[id]) {
            if (params_.verbose_planner) {
              ROS_INFO_STREAM("[MR Global Planner] Agent "
                              << agent_id << " in collision with another path");
            }
            return true;
          }
        }
      }
    }
  }

  // Collision free
  return false;
}

bool MultiRobotGlobalPlanner::computePathToWaypoint(
    const Eigen::Vector4d &start_pose, const int agent_id) {
  // Get the position of the agent in the world frame. This is the start
  // position for planning
  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Planning for agent " << agent_id);
  }
  Eigen::Vector3d start = start_pose.head(3);

  // Get goal position for planning
  int index_waypoint = agents_[agent_id].waypoint_num;
  Eigen::Vector3d goal = waypoints_lists_[agent_id][index_waypoint];

  // Set up the right robot radius
  rrt_->setRobotRadius(robot_radii_[agent_id]);

  // Perform planning: first try straight line connection. If this is not
  // feasible, then plan with OMPL
  std::vector<Eigen::Vector3d> path_vector;
  int n_steps =
      std::min(20, static_cast<int>(std::ceil((goal - start).norm() / 0.5)));
  if (!rrt_->validStraightLine(start, goal, n_steps, path_vector)) {
    ROS_WARN_STREAM(
        "[MR Global Planner] Straight line connection not possible "
        "for agent "
        << agent_id);

    // Try to plan with OMPL here
    mav_msgs::EigenTrajectoryPoint start_point, goal_point;
    start_point.position_W = start;
    goal_point.position_W = goal;

    if (!checkOptimisticMapCollision(goal_point.position_W, agent_id)) {
      ROS_ERROR_STREAM("[MR Global Planner] Goal number "
                       << agents_[agent_id].waypoint_num - 1 << " for agent "
                       << agent_id << " is in occupied position");
      agents_[agent_id].valid_path = false;
      return false;
    }

    // Figure out map bounds!
    if (params_.verbose_planner) {
      ROS_INFO_STREAM(
          "[MR Planner] Map bounds: "
          << params_.lower_bound.transpose() << " to "
          << params_.upper_bound.transpose() << " size: "
          << (params_.upper_bound - params_.lower_bound).transpose());
    }

    // Inflate the bounds a bit.
    Eigen::Vector3d inflation(params_.bounding_box_inflation,
                              params_.bounding_box_inflation, 0.0);
    // Don't in flate in z. ;)
    rrt_->setBounds(params_.lower_bound - inflation,
                    params_.upper_bound + inflation);
    rrt_->setupProblem(start, goal);

    if (params_.verbose_planner) {
      ROS_INFO("[MR Global Planner] Successfully set up problem");
      ROS_INFO_STREAM("Start: [" << start.x() << ", " << start.y() << ", "
                                 << start.z() << "]");
      ROS_INFO_STREAM("Goal: [" << goal.x() << ", " << goal.y() << ", "
                                << goal.z() << "]");
    }

    mav_msgs::EigenTrajectoryPoint::Vector path_vector_eigen;
    if (!rrt_->getPathBetweenWaypoints(start_point, goal_point,
                                       &path_vector_eigen)) {
      ROS_ERROR_STREAM(
          "[MR Global Planner] Could not find path with OMPL for"
          " agent "
          << agent_id);
      agents_[agent_id].valid_path = false;
      return false;
    }

    // Store the path from OMPL
    path_vector_eigen[0].setFromYaw(start_pose(3));
    GlobalPath global_path = getGlobalPathFromEigenVector(path_vector_eigen);

    // Interpolate global path
    GlobalPath interpolated_global_path;
    if (params_.interpolator == Interpolator::Polynomial) {
      polynomial_interpolator_->interpolate(global_path,
                                            &interpolated_global_path);
    } else if (params_.interpolator == Interpolator::Ramp) {
      ramp_interpolator_->interpolate(global_path, &interpolated_global_path);
    }

    agents_[agent_id].global_path = interpolated_global_path;
    agents_[agent_id].valid_path = true;

    publishPathSingleAgents(agent_id);
    return true;
  }

  // Store the straight line path
  if (params_.verbose_planner) {
    ROS_INFO_STREAM(
        "[MR Global Planner] Found straight line connection for "
        "agent "
        << agent_id);
  }
  GlobalPath global_path =
      getGlobalPathFromStdVector(path_vector, start_pose(3));

  // In the case of straight line connection, there is no need for interpolation
  agents_[agent_id].global_path = global_path;
  agents_[agent_id].valid_path = true;

  publishPathSingleAgents(agent_id);
  return true;
}

bool MultiRobotGlobalPlanner::computeConstrainedPathToWaypoint(
    const Eigen::Vector4d &start_pose, const int agent_id) {
  // Check: if we are planning for the first agent (id = 0), then we plan as
  // usual - without any constraints
  if (agent_id == 0) {
    return computePathToWaypoint(start_pose, agent_id);
  }

  // Else: we are planning for the other (slave) agents

  // Get the position of the agent in the world frame. This is the start
  // position for planning
  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Global Planner] Planning constrained path for agent "
                    << agent_id);
  }
  Eigen::Vector3d start = start_pose.head(3);

  // Get goal position for planning
  int64_t index_waypoint = agents_[agent_id].waypoint_num;
  Eigen::Vector3d goal = waypoints_lists_[agent_id][index_waypoint];

  // Set up the right robot radius
  rrt_->setRobotRadius(robot_radii_[agent_id]);

  // Perform planning: first try straight line connection. If this is not
  // feasible, then plan with OMPL
  std::vector<Eigen::Vector3d> path_vector;
  int n_steps =
      std::min(20, static_cast<int>(std::ceil((goal - start).norm() / 0.5)));

  // Try straight line connection - first select all the paths to be checked
  std::vector<std::vector<Eigen::Vector3d> > constrain_paths(agent_id);
  for (int id = 0; id < agent_id; ++id) {
    for (GlobalPath::iterator it = agents_[id].global_path.begin();
         it < agents_[id].global_path.end(); ++it) {
      constrain_paths[id].push_back((*it).head(3));
    }
  }

  if (rrt_->validConstrainedStraightLine(start, goal, n_steps, constrain_paths,
                                         path_vector)) {
    // Store the straight line path
    if (params_.verbose_planner) {
      ROS_INFO_STREAM(
          "[MR Global Planner] Found straight line connection for "
          "agent "
          << agent_id);
    }
    GlobalPath global_path =
        getGlobalPathFromStdVector(path_vector, start_pose(3));

    // For straight line connection, there is no need for interpolation
    agents_[agent_id].global_path = global_path;
    agents_[agent_id].valid_path = true;

    publishPathSingleAgents(agent_id);
    return true;
  }

  // Else Straight line planning was not successful
  ROS_WARN_STREAM(
      "[MR Global Planner] Straight line connection not possible "
      "for agent "
      << agent_id);

  // Try to plan with OMPL here
  mav_msgs::EigenTrajectoryPoint start_point, goal_point;
  start_point.position_W = start;
  goal_point.position_W = goal;

  if (!checkOptimisticMapCollision(goal_point.position_W, agent_id)) {
    ROS_ERROR_STREAM("[MR Global Planner] Goal number "
                     << agents_[agent_id].waypoint_num - 1 << " for agent "
                     << agent_id << " is in occupied position");
    agents_[agent_id].valid_path = false;
    return false;
  }

  // Figure out map bounds!
  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[MR Planner] Map bounds: "
                    << params_.lower_bound.transpose() << " to "
                    << params_.upper_bound.transpose() << " size: "
                    << (params_.upper_bound - params_.lower_bound).transpose());
  }

  // Inflate the bounds a bit.
  Eigen::Vector3d inflation(params_.bounding_box_inflation,
                            params_.bounding_box_inflation, 0.0);
  // Don't in flate in z. ;)
  rrt_->setBounds(params_.lower_bound - inflation,
                  params_.upper_bound + inflation);
  rrt_->setConstraintPaths(constrain_paths);
  rrt_->setupProblem(start, goal, true);

  if (params_.verbose_planner) {
    ROS_INFO("[MR Global Planner] Successfully set up problem");
    ROS_INFO_STREAM("Start: [" << start.x() << ", " << start.y() << ", "
                               << start.z() << "]");
    ROS_INFO_STREAM("Goal: [" << goal.x() << ", " << goal.y() << ", "
                              << goal.z() << "]");
  }

  mav_msgs::EigenTrajectoryPoint::Vector path_vector_eigen;
  if (!rrt_->getPathBetweenWaypoints(start_point, goal_point,
                                     &path_vector_eigen)) {
    ROS_ERROR_STREAM(
        "[MR Global Planner] Could not find path with OMPL for"
        " agent "
        << agent_id);
    agents_[agent_id].valid_path = false;
    return false;
  }

  // Store the path from OMPL
  path_vector_eigen[0].setFromYaw(start_pose(3));
  GlobalPath global_path = getGlobalPathFromEigenVector(path_vector_eigen);

  // Interpolate global path
  GlobalPath interpolated_global_path;
  if (params_.interpolator == Interpolator::Polynomial) {
    polynomial_interpolator_->interpolate(global_path,
                                          &interpolated_global_path);
  } else if (params_.interpolator == Interpolator::Ramp) {
    ramp_interpolator_->interpolate(global_path, &interpolated_global_path);
  }

  agents_[agent_id].global_path = interpolated_global_path;
  agents_[agent_id].valid_path = true;

  publishPathSingleAgents(agent_id);
  return true;
}

void MultiRobotGlobalPlanner::computeMapBounds(
    Eigen::Vector3d *lower_bound, Eigen::Vector3d *upper_bound) const {
  if (esdf_map_ && !params_.optimistic) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

bool MultiRobotGlobalPlanner::checkOptimisticMapCollision(
    const Eigen::Vector3d &robot_position, const int agent_id) {
  // This function returns false if in collision
  voxblox::Layer<voxblox::TsdfVoxel> *layer =
      voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
  voxblox::HierarchicalIndexMap block_voxel_list;

  // Extract all the voxels in the sphere for collision checks
  voxblox::utils::getSphereAroundPoint(
      *layer, robot_point, robot_radii_[agent_id], &block_voxel_list);

  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
       block_voxel_list) {
    // Get block -- only already existing blocks are in the list.
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
        layer->getBlockPtrByIndex(kv.first);

    if (!block_ptr) {
      continue;
    }

    for (const voxblox::VoxelIndex &voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        return false;
      }
      const voxblox::TsdfVoxel &tsdf_voxel =
          block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (tsdf_voxel.distance < 0.0f) {
        return false;
      }
    }
  }

  // No collision if nothing in the sphere had a negative or 0 distance.
  // Unknown space is unoccupied, since this is a very optimistic global
  // planner.
  return true;
}

double MultiRobotGlobalPlanner::getMapDistance(
    const Eigen::Vector3d &position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    if (params_.verbose_planner) {
      ROS_WARN("[MR Global Planner] Invalid ESDF map");
    }
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    if (params_.verbose_planner) {
      ROS_INFO("[MR Global Planner] Unknown location");
    }
    return 0.0;
  }
  return distance;
}

bool MultiRobotGlobalPlanner::getPoseAgentId(const size_t agent_id,
                                             Eigen::Vector4d &pose) const {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(world_frame_,
                                 agent_frame_ + std::to_string(agent_id),
                                 ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    if (!agents_[agent_id].transformations_initialized_O_A ||
        !agents_[agent_id].transformations_initialized_M_O ||
        !agents_[agent_id].transformations_initialized_W_M) {
      ROS_WARN_STREAM("[MR Global Planner] Transformations for agent "
                      << agent_id << " not initialized");
      ROS_ERROR("[MR Global Planner] %s", ex.what());
      return false;
    } else {
      ROS_WARN_STREAM_THROTTLE(10,
                               "[MR Global Planner] Using transformation "
                               "to pose of agent "
                                   << agent_id);
      Eigen::Matrix4d T_W_A(agents_[agent_id].T_W_M * agents_[agent_id].T_M_O *
                            agents_[agent_id].T_O_A);
      pose.head(3) = T_W_A.block<3, 1>(0, 3);

      tf::Quaternion quat_tf;
      tf::quaternionEigenToTF(Eigen::Quaterniond(T_W_A.block<3, 3>(0, 0)),
                              quat_tf);
      pose(3) = tf::getYaw(quat_tf);
      return true;
    }
  }

  Eigen::Vector3d position;
  tf::vectorTFToEigen(transform.getOrigin(), position);

  pose.head(3) = position;
  pose(3) = tf::getYaw(transform.getRotation());
  return true;
}

bool MultiRobotGlobalPlanner::isGoalReachedAgent(
    const size_t agent_id, const Eigen::Vector4d &agent_pose) const {
  // Get the index of the current goal
  uint64_t index_wp = agents_[agent_id].waypoint_num;
  Eigen::Vector3d goal(waypoints_lists_[agent_id][index_wp]);

  return (agent_pose.head(3) - goal).norm() < params_.goal_threshold;
}

bool MultiRobotGlobalPlanner::publishPathsToAgents() const {
  // Iterate over the agents and publish the paths
  for (size_t id = 0; id < num_agents_; ++id) {
    if (!publishPathSingleAgents(id)) {
      return false;
    }
  }
  return true;
}

bool MultiRobotGlobalPlanner::publishPathSingleAgents(
    const int agent_id) const {
  // Get the transformation T_w_o (odometry to world)
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(world_frame_,
                                 odometry_frame_ + std::to_string(agent_id),
                                 ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("[MR Global Planner] %s", ex.what());
    return false;
  }

  // Store the transformation in Eigen format
  Eigen::Quaterniond quaternion_eigen;
  tf::quaternionTFToEigen(transform.getRotation(), quaternion_eigen);

  Eigen::Matrix4d T_w_o;
  T_w_o.block(0, 3, 3, 1) << transform.getOrigin().getX(),
      transform.getOrigin().getY(), transform.getOrigin().getZ();
  T_w_o.block(0, 0, 3, 3) << quaternion_eigen.normalized().toRotationMatrix();
  T_w_o.block(3, 0, 1, 4) << 0.0, 0.0, 0.0, 1.0;

  // Store the inverse
  Eigen::Matrix4d T_o_w = T_w_o.inverse();

  // Now iterate over the poses in the path, transform them in odometry frame
  nav_msgs::Path path_agent;
  path_agent.header.frame_id = odometry_frame_ + std::to_string(agent_id);
  path_agent.header.stamp = ros::Time::now();
  path_agent.header.seq = 0;

  for (int i = 0; i < agents_[agent_id].global_path.size(); ++i) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = odometry_frame_ + std::to_string(agent_id);
    pose_stamped.header.stamp =
        ros::Time(static_cast<double>(i) * dynamic_params_.sampling_dt);
    pose_stamped.header.seq = i;

    Eigen::Vector3d position = agents_[agent_id].global_path[i].head(3);
    double yaw = agents_[agent_id].global_path[i](3);

    Eigen::Matrix3d Rot(Eigen::Matrix3d::Zero());
    Rot.block(0, 0, 1, 2) << std::cos(yaw), -std::sin(yaw);
    Rot.block(1, 0, 1, 2) << std::sin(yaw), std::cos(yaw);
    Rot(2, 2) = 1.0;

    Eigen::Matrix3d Rot_o_w(T_o_w.block(0, 0, 3, 3));
    Eigen::Vector3d transformed_posit(Rot_o_w * position +
                                      T_o_w.block(0, 3, 3, 1));
    Eigen::Quaterniond transformed_orient(Rot_o_w * Rot);

    tf::pointEigenToMsg(transformed_posit, pose_stamped.pose.position);
    tf::quaternionEigenToMsg(transformed_orient, pose_stamped.pose.orientation);
    path_agent.poses.push_back(pose_stamped);
  }

  // Publish
  if (global_paths_pubs_[agent_id].getNumSubscribers() > 0) {
    ROS_INFO_STREAM("[MR Global Planner] Published path to agent " << agent_id);
    global_paths_pubs_[agent_id].publish(path_agent);
  }

  return true;
}

void MultiRobotGlobalPlanner::sendStopCommand(const int agent_id) const {
  // Send an empty path to the local planner
  nav_msgs::Path path_agent;
  path_agent.header.frame_id = odometry_frame_ + std::to_string(agent_id);
  path_agent.header.stamp = ros::Time::now();
  path_agent.header.seq = 0;

  if (global_paths_pubs_[agent_id].getNumSubscribers() > 0) {
    global_paths_pubs_[agent_id].publish(path_agent);
  }
  if (params_.verbose_planner) {
    ROS_WARN_STREAM(
        "[MR Global Planner] Published empty trajectory to the "
        "local planner for agent "
        << agent_id);
  }
}

void MultiRobotGlobalPlanner::callStopSrvLocalPlanner(const int agent_id) {
  std::string stop_srv_name = "/stop_agent_" + std::to_string(agent_id);
  std_srvs::Empty empty_srv;
  ros::ServiceClient client_stop_srv =
      nh_.serviceClient<std_srvs::Empty>(stop_srv_name);
  if (client_stop_srv.exists()) {
    client_stop_srv.call(empty_srv);
  } else {
    ROS_WARN_STREAM("[MR Global Planner]  Service " << stop_srv_name
                                                    << " does not exist");
  }
}

GlobalPath MultiRobotGlobalPlanner::getGlobalPathFromEigenVector(
    const mav_msgs::EigenTrajectoryPoint::Vector &path_eigen_vector) const {
  GlobalPath global_path;
  global_path.push_back(Eigen::Vector4d(
      path_eigen_vector[0].position_W(0), path_eigen_vector[0].position_W(1),
      path_eigen_vector[0].position_W(2), path_eigen_vector[0].getYaw()));

  for (size_t i = 1; i < path_eigen_vector.size() - 1; ++i) {
    mav_msgs::EigenTrajectoryPoint point_curr = path_eigen_vector[i];
    mav_msgs::EigenTrajectoryPoint point_next = path_eigen_vector[i + 1];
    Eigen::Vector3d direction =
        (point_next.position_W - point_curr.position_W).normalized();

    global_path.push_back(Eigen::Vector4d(
        path_eigen_vector[i].position_W(0), path_eigen_vector[i].position_W(1),
        path_eigen_vector[i].position_W(2),
        std::atan2(direction(1), direction(0))));
  }
  double final_yaw = global_path.back()(3);
  global_path.push_back(Eigen::Vector4d(path_eigen_vector.back().position_W(0),
                                        path_eigen_vector.back().position_W(1),
                                        path_eigen_vector.back().position_W(2),
                                        final_yaw));

  return global_path;
}

GlobalPath MultiRobotGlobalPlanner::getGlobalPathFromStdVector(
    const std::vector<Eigen::Vector3d> &path_std_vector,
    const double start_yaw) const {
  GlobalPath global_path;
  global_path.push_back(Eigen::Vector4d(path_std_vector[0](0),
                                        path_std_vector[0](1),
                                        path_std_vector[0](2), start_yaw));

  for (size_t i = 1; i < path_std_vector.size() - 1; ++i) {
    Eigen::Vector3d direction =
        (path_std_vector[i + 1] - path_std_vector[i]).normalized();

    global_path.push_back(Eigen::Vector4d(
        path_std_vector[i](0), path_std_vector[i](1), path_std_vector[i](2),
        std::atan2(direction(1), direction(0))));
  }
  global_path.push_back(global_path.back());
  return global_path;
}

inline bool MultiRobotGlobalPlanner::isAgentSentHome(const int agent_id) const {
  return agents_[agent_id].home_triggered;
}

bool MultiRobotGlobalPlanner::checkForceReplanning() {
  bool force_replanning = false;
  if (clock_replanning_ > 0.0) {
    force_replanning =
        std::fmod(timer_dt_ * iter_timer_, clock_replanning_) == 0.0;

    if (force_replanning) {
      if (params_.verbose_planner) {
        ROS_INFO("[MR Global Planner] Forcing replanning with updated map");
      }
      iter_timer_ = 0;
    }
  }

  return force_replanning;
}

void MultiRobotGlobalPlanner::publishMarkerWaypoints() const {
  visualization_msgs::MarkerArray waypoints_array;

  for (int id = 0; id < num_agents_; ++id) {
    visualization_msgs::Marker waypoints;

    waypoints.header.frame_id = world_frame_;
    waypoints.header.stamp = ros::Time::now();
    waypoints.header.seq = 0;

    waypoints.action = visualization_msgs::Marker::MODIFY;
    waypoints.lifetime = ros::Duration(0);

    waypoints.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints.ns = "waypoints_" + std::to_string(id);
    waypoints.id = id;
    waypoints.scale.x = waypoints.scale.y = waypoints.scale.z =
        0.20 * params_.scale_factor_visualization;

    waypoints.points.reserve(waypoints_lists_[id].size());
    for (int i = 0; i < waypoints_lists_[id].size(); ++i) {
      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(waypoints_lists_[id][i], point_msg);
      waypoints.points.push_back(point_msg);
    }

    switch (id) {
      case 0:
        waypoints.color = mrp::Color::Red();
        waypoints.color.a = 0.7;
        break;
      case 1:
        waypoints.color = mrp::Color::Green();
        waypoints.color.a = 0.7;
        break;
      case 2:
        waypoints.color = mrp::Color::Blue();
        waypoints.color.a = 0.7;
        break;
      case 3:
        waypoints.color = mrp::Color::Orange();
        waypoints.color.a = 0.7;
        break;
      default:
        waypoints.color = mrp::Color::Yellow();
        waypoints.color.a = 0.7;
        break;
    }

    waypoints_array.markers.push_back(waypoints);
  }

  // Add the limits
  visualization_msgs::Marker borders;
  borders.header.frame_id = world_frame_;
  borders.header.stamp = ros::Time::now();
  borders.header.seq = 0;

  borders.action = visualization_msgs::Marker::MODIFY;
  borders.lifetime = ros::Duration(0);

  borders.type = visualization_msgs::Marker::LINE_STRIP;
  borders.color = mrp::Color::Red();
  borders.color.a = 0.5;
  borders.ns = "borders";
  borders.id = num_agents_ + 1;
  borders.scale.x = 0.1 * params_.scale_factor_visualization;

  geometry_msgs::Point p1;
  p1.x = params_.lower_bound.x();
  p1.y = params_.lower_bound.y();
  p1.z = params_.lower_bound.z();

  geometry_msgs::Point p2;
  p2.x = params_.lower_bound.x();
  p2.y = params_.lower_bound.y();
  p2.z = params_.upper_bound.z();

  geometry_msgs::Point p3;
  p3.x = params_.lower_bound.x();
  p3.y = params_.upper_bound.y();
  p3.z = params_.lower_bound.z();

  geometry_msgs::Point p4;
  p4.x = params_.lower_bound.x();
  p4.y = params_.upper_bound.y();
  p4.z = params_.upper_bound.z();

  geometry_msgs::Point p5;
  p5.x = params_.upper_bound.x();
  p5.y = params_.lower_bound.y();
  p5.z = params_.lower_bound.z();

  geometry_msgs::Point p6;
  p6.x = params_.upper_bound.x();
  p6.y = params_.lower_bound.y();
  p6.z = params_.upper_bound.z();

  geometry_msgs::Point p7;
  p7.x = params_.upper_bound.x();
  p7.y = params_.upper_bound.y();
  p7.z = params_.lower_bound.z();

  geometry_msgs::Point p8;
  p8.x = params_.upper_bound.x();
  p8.y = params_.upper_bound.y();
  p8.z = params_.upper_bound.z();

  // Insert point in the right order
  borders.points.push_back(p1);
  borders.points.push_back(p2);
  borders.points.push_back(p4);
  borders.points.push_back(p3);
  borders.points.push_back(p1);
  borders.points.push_back(p5);
  borders.points.push_back(p6);
  borders.points.push_back(p2);
  borders.points.push_back(p6);
  borders.points.push_back(p8);
  borders.points.push_back(p4);
  borders.points.push_back(p3);
  borders.points.push_back(p7);
  borders.points.push_back(p8);
  borders.points.push_back(p6);
  borders.points.push_back(p5);
  borders.points.push_back(p7);
  waypoints_array.markers.push_back(borders);

  // Publish markers
  waypoints_marker_pub_.publish(waypoints_array);
}

void MultiRobotGlobalPlanner::publishMarkerPaths() const {
  visualization_msgs::MarkerArray marker_array;

  for (int id = 0; id < num_agents_; ++id) {
    // First, add the path
    visualization_msgs::Marker path;

    path.header.frame_id = world_frame_;
    path.header.stamp = ros::Time::now();
    path.header.seq = 0;

    path.action = visualization_msgs::Marker::MODIFY;
    path.lifetime = ros::Duration(0);

    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.ns = "path_" + std::to_string(id);
    path.id = id;
    path.scale.x = path.scale.y = path.scale.z =
        0.05 * params_.scale_factor_visualization;

    path.points.reserve(agents_[id].global_path.size());
    for (int i = 0; i < agents_[id].global_path.size(); ++i) {
      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(agents_[id].global_path[i].head(3), point_msg);
      path.points.push_back(point_msg);
    }

    // Then, add the waypoints
    visualization_msgs::Marker waypoints;

    waypoints.header = path.header;
    waypoints.action = visualization_msgs::Marker::MODIFY;
    waypoints.lifetime = ros::Duration(0);

    waypoints.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints.ns = "waypoints_" + std::to_string(id);
    waypoints.id = id;
    waypoints.scale.x = waypoints.scale.y = waypoints.scale.z =
        0.125 * params_.scale_factor_visualization;

    waypoints.points.reserve(agents_[id].global_path.size());
    for (int i = 0; i < agents_[id].global_path.size(); ++i) {
      geometry_msgs::Point point_msg;
      tf::pointEigenToMsg(agents_[id].global_path[i].head(3), point_msg);
      waypoints.points.push_back(point_msg);
    }

    switch (id) {
      case 0:
        path.color = mrp::Color::Red();
        waypoints.color = mrp::Color::Red();
        break;
      case 1:
        path.color = mrp::Color::Green();
        waypoints.color = mrp::Color::Green();
        break;
      case 2:
        path.color = mrp::Color::Blue();
        waypoints.color = mrp::Color::Blue();
        break;
      case 3:
        path.color = mrp::Color::Orange();
        waypoints.color = mrp::Color::Orange();
        break;
      default:
        path.color = mrp::Color::Yellow();
        waypoints.color = mrp::Color::Yellow();
        break;
    }

    // Store stuff
    marker_array.markers.push_back(path);
    marker_array.markers.push_back(waypoints);
  }
  // Publish markers
  path_marker_pub_.publish(marker_array);
}

}  // end namespace mrp
