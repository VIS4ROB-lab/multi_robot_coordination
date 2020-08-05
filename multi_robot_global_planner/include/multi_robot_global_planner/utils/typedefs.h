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
 * @brief Different optimization objectives
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

  Eigen::Vector3d lower_bound;
  Eigen::Vector3d upper_bound;
  double bounding_box_inflation;

  // Whether to trust an approximate solution (i.e., not necessarily reaching
  // the exact goal state).
  bool trust_approx_solution;
  bool optimistic;
  bool use_distance_threshold;
  OptimizationObjective optimization_objective;
  double distance_threshold;
  RrtPlannerType planner_type;

  double goal_threshold;
  AltitudeObjectiveParams altitude_obj_params;

  // Type of planning
  PlanningStrategy planning_strategy;

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
