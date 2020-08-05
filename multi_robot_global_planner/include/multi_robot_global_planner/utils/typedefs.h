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
enum OptimizationObjective {
    kDefault = 0,
    kAltitude
};

/**
 * @brief Ways of interpolating the path
 */
enum Interpolator {
    Polynomial = 0,
    Ramp
};

/**
 * @brief Methods to plan
 */
enum PlanningStrategy {
    OneByOne = 0,
    OneShot,
    ConstrainedOneByOne
};

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
  double planning_height; // Planning height above ground
  double num_seconds_to_plan;
  double safety_factor; // Safety factor for cross-agents collisions checking
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

typedef std::vector<Eigen::Vector3d> WaypointsList;
typedef std::vector<Eigen::Vector4d> GlobalPath;

} // end namespace mrp