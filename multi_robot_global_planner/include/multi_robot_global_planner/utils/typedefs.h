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
 * @brief  Useful typedefs
 * @date   01.08.2019
 */

#include <Eigen/Eigen>

namespace mrp {

/**
 * @brief Available planners
 */
enum RrtPlannerType {
  kRrtConnect = 0,
  kRrtStar,
  kInformedRrtStar,
  kBitStar,
  kPrm
};

/**
 * @brief Different optimization objectives. Additional custom objectives can
 *        be easily added.
 */
enum OptimizationObjective { kDefault = 0, kAltitude };

/**
 * @brief Ways of interpolating the path
 */
enum Interpolator { Polynomial = 0, Ramp };

/**
 * @brief Methods to plan
 */
enum PlanningStrategy { OneByOne = 0, OneShot, ConstrainedOneByOne };

/**
 * @brief Altitude objective parameters
 */
struct AltitudeObjectiveParams {
  double alpha;
  double beta;
};

/**
 * @brief Struct for the parameters for the OMPL solver
 */
struct GlobalPlannerParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Planning
  double planning_height;  // Planning height above ground
  double num_seconds_to_plan;
  double safety_factor;  // Safety factor for cross-agents collisions checking
  bool simplify_solution;

  // Bounding box defining the space useful for planning
  Eigen::Vector3d lower_bound;
  Eigen::Vector3d upper_bound;
  double bounding_box_inflation;

  // Whether to trust an approximate solution (i.e., not necessarily reaching
  // the exact goal state).
  bool trust_approx_solution;

  // Whether to consider unknown space to be free
  bool optimistic;

  // Whether to use a min cost treshold on planning. If true, the planner
  // returns the first solution whose cost is lower or equal this threshold.
  bool use_distance_threshold;
  double distance_threshold;

  // Optimization objective, type of planner and planning strategy
  OptimizationObjective optimization_objective;
  PlanningStrategy planning_strategy;
  RrtPlannerType planner_type;

  // Auxiliary used only for altitude objective
  AltitudeObjectiveParams altitude_obj_params;

  // Defines the tolerance on the goal position when planning
  double goal_threshold;

  // Type of interpolator
  Interpolator interpolator;

  // Miscellaneous
  bool visualize;
  double scale_factor_visualization;
  bool verbose_planner;
};

/**
 * @brief Dynamic constraints for global planner
 */
struct DynamicParams {
  double v_max;
  double a_max;

  double v_yaw_max;
  double a_yaw_max;

  double sampling_dt;
};

/**
 * @brief Bunch of useful typedefs
 */
typedef std::vector<Eigen::Vector3d> WaypointsList;
typedef std::vector<Eigen::Vector4d> GlobalPath;

/**
 * @brief Struct that contains all the information about an agent's state
 */
struct Agent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor
   */
  Agent()
      : waypoint_num(0),
        valid_path(false),
        reached_all_goals(false),
        move_next_waypoint(false),
        home_triggered(false),
        T_O_A(Eigen::Matrix4d::Identity()),
        T_W_M(Eigen::Matrix4d::Identity()),
        T_M_O(Eigen::Matrix4d::Identity()),
        transformations_initialized_O_A(false),
        transformations_initialized_M_O(false),
        transformations_initialized_W_M(false) {}

  // Collision avoidance
  double robot_radius;

  // Current global path for the agent
  GlobalPath global_path;

  // Storage for the current waypoint number in the waypoint list. It stores
  // the current waypoint list index to plan to
  uint64_t waypoint_num;

  // Checks: it tells us if the agent has a valid path
  bool valid_path;
  bool reached_all_goals;
  bool move_next_waypoint;

  // Storage for the starting positions of the agents, in case the return
  // home is triggered
  Eigen::Vector3d home_position;
  bool home_triggered;

  // Storage for the transformations
  // Reference frame convention for each agent:
  // - W : world
  // - M : map
  // - O : odom
  // - A : agent
  // Convention: T_B_A --> from A to B
  Eigen::Matrix4d T_O_A;  // this is from odometry information
  Eigen::Matrix4d T_W_M;  // this is from pose graph backend
  Eigen::Matrix4d T_M_O;  // this is from pose graph backend

  bool transformations_initialized_O_A;
  bool transformations_initialized_M_O;
  bool transformations_initialized_W_M;
};

}  // end namespace mrp
