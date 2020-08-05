#include "multi_robot_global_planner/voxblox_ompl_rrt.h"

namespace mrp {

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      optimistic_(true),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()) {}

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               const int num_agents)
    : nh_(nh),
      nh_private_(nh_private),
      optimistic_(true),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()),
      multi_agent_problem_setup_(num_agents) {}

void VoxbloxOmplRrt::setBounds(const Eigen::Vector3d &lower_bound,
                               const Eigen::Vector3d &upper_bound) {
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void VoxbloxOmplRrt::setTsdfLayer(
    voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  CHECK_NOTNULL(tsdf_layer_);
  voxel_size_ = tsdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setEsdfLayer(
    voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
  esdf_layer_ = esdf_layer;
  CHECK_NOTNULL(esdf_layer_);
  voxel_size_ = esdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setConstraintPaths(
    const std::vector<std::vector<Eigen::Vector3d> > &constraint_paths) {
  constraint_paths_ = constraint_paths;
}

void VoxbloxOmplRrt::setupProblem(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &goal,
                                  const bool constrained_planning) {
  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    if (constrained_planning) {
      problem_setup_.setTsdfVoxbloxCollisionCheckingHierarchical(
          robot_radius_, params_.safety_factor, constraint_paths_, tsdf_layer_);
    } else {
      problem_setup_.setTsdfVoxbloxCollisionChecking(robot_radius_,
                                                     tsdf_layer_);
    }
  } else {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_.setEsdfVoxbloxCollisionChecking(robot_radius_, esdf_layer_);
  }

  if (params_.optimization_objective == OptimizationObjective::kDefault) {
    if (!params_.use_distance_threshold) {
      problem_setup_.setDefaultObjective();
    } else {
      problem_setup_.setLenghtOptimizationObjective(params_.distance_threshold *
                                                    (goal - start).norm());
    }
  } else if (params_.optimization_objective ==
             OptimizationObjective::kAltitude) {
    problem_setup_.setConstantAltitudeObjective(
        start.z(), params_.altitude_obj_params.alpha,
        params_.altitude_obj_params.beta);
  }

  if (params_.planner_type == kRrtConnect) {
    problem_setup_.setRrtConnect();
  } else if (params_.planner_type == kRrtStar) {
    problem_setup_.setRrtStar();
  } else if (params_.planner_type == kInformedRrtStar) {
    problem_setup_.setInformedRrtStar();
  } else if (params_.planner_type == kPrm) {
    problem_setup_.setPrm();
  } else if (params_.planner_type == kBitStar) {
    problem_setup_.setBitStar();
  } else {
    problem_setup_.setDefaultPlanner();
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());
    bounds.setLow(2, lower_bound_.z());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());
    bounds.setHigh(2, upper_bound_.z());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::mrp::RStateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

void VoxbloxOmplRrt::setupReturnHomeProblem(const Eigen::Vector3d &start,
                                            const Eigen::Vector3d &goal,
                                            const bool constrained_planning) {
  if (constrained_planning) {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_.setEsdfVoxbloxCollisionCheckingHierarchical(
        robot_radius_, params_.safety_factor, constraint_paths_, esdf_layer_);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_.setEsdfVoxbloxCollisionChecking(robot_radius_, esdf_layer_);
  }

  if (!params_.use_distance_threshold) {
    problem_setup_.setDefaultObjective();
  } else {
    problem_setup_.setLenghtOptimizationObjective(params_.distance_threshold *
                                                  (goal - start).norm());
  }

  if (params_.planner_type == kRrtConnect) {
    problem_setup_.setRrtConnect();
  } else if (params_.planner_type == kRrtStar) {
    problem_setup_.setRrtStar();
  } else if (params_.planner_type == kInformedRrtStar) {
    problem_setup_.setInformedRrtStar();
  } else if (params_.planner_type == kPrm) {
    problem_setup_.setPrm();
  } else if (params_.planner_type == kBitStar) {
    problem_setup_.setBitStar();
  } else {
    problem_setup_.setDefaultPlanner();
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());
    bounds.setLow(2, lower_bound_.z());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());
    bounds.setHigh(2, upper_bound_.z());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::mrp::RStateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

void VoxbloxOmplRrt::setupMultiAgentProblem(const int num_agents) {
  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    multi_agent_problem_setup_.setTsdfVoxbloxCollisionCheckingCompoundState(
        robot_radius_, params_.safety_factor, tsdf_layer_);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    multi_agent_problem_setup_.setEsdfVoxbloxCollisionChecking(robot_radius_,
                                                               esdf_layer_);
  }

  multi_agent_problem_setup_.setDefaultObjective();
  if (params_.planner_type == kRrtConnect) {
    multi_agent_problem_setup_.setRrtConnect();
  } else if (params_.planner_type == kRrtStar) {
    multi_agent_problem_setup_.setRrtStar();
  } else if (params_.planner_type == kInformedRrtStar) {
    multi_agent_problem_setup_.setInformedRrtStar();
  } else if (params_.planner_type == kPrm) {
    multi_agent_problem_setup_.setPrm();
  } else if (params_.planner_type == kBitStar) {
    multi_agent_problem_setup_.setBitStar();
  } else {
    multi_agent_problem_setup_.setDefaultPlanner();
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());
    bounds.setLow(2, lower_bound_.z());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());
    bounds.setHigh(2, upper_bound_.z());

    // Define start and goal positions.
    for (size_t id = 0; id < num_agents; ++id) {
      multi_agent_problem_setup_.getGeometricComponentStateSpace()
          ->as<ompl::base::CompoundStateSpace>()
          ->as<ompl::mrp::RStateSpace>(id)
          ->setBounds(bounds);
    }
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  multi_agent_problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

bool VoxbloxOmplRrt::getPathBetweenWaypoints(
    const mav_msgs::EigenTrajectoryPoint &start,
    const mav_msgs::EigenTrajectoryPoint &goal,
    mav_msgs::EigenTrajectoryPointVector *solution) {
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  if (problem_setup_.solve(params_.num_seconds_to_plan)) {
    if (problem_setup_.haveExactSolutionPath()) {
      // Simplify and print.
      // TODO(helenol): look more into this. Appears to actually prefer more
      // vertices with presumably shorter total path length, which is
      // detrimental to polynomial planning.
      if (params_.simplify_solution) {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner) {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
    } else if (params_.trust_approx_solution &&
               problem_setup_.haveSolutionPath()) {
      ROS_WARN("OMPL found an approximated solution.");
    } else {
      ROS_WARN("OMPL planning failed.");
      return false;
    }
  }

  if (problem_setup_.haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), solution);
    return true;
  }
  return false;
}

void VoxbloxOmplRrt::setupFromStartAndGoal(
    const mav_msgs::EigenTrajectoryPoint &start,
    const mav_msgs::EigenTrajectoryPoint &goal) {
  if (params_.planner_type == kPrm) {
    std::dynamic_pointer_cast<ompl::geometric::PRM>(problem_setup_.getPlanner())
        ->clearQuery();
  } else {
    problem_setup_.clear();
  }

  ompl::base::ScopedState<ompl::mrp::RStateSpace> start_ompl(
      problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::mrp::RStateSpace> goal_ompl(
      problem_setup_.getSpaceInformation());

  start_ompl->values[0] = start.position_W.x();
  start_ompl->values[1] = start.position_W.y();
  start_ompl->values[2] = start.position_W.z();

  goal_ompl->values[0] = goal.position_W.x();
  goal_ompl->values[1] = goal.position_W.y();
  goal_ompl->values[2] = goal.position_W.z();

  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl, voxel_size_);
}

void VoxbloxOmplRrt::setupMultiAgentFromStartAndGoal(
    const mav_msgs::EigenTrajectoryPointVector &starts,
    const mav_msgs::EigenTrajectoryPointVector &goals) {
  // Get the number of agents we have here
  size_t num_agents = starts.size();

  // Now create the problem for multiple agents
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> start_ompl(
      multi_agent_problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal_ompl(
      multi_agent_problem_setup_.getSpaceInformation());

  for (size_t id = 0; id < num_agents; ++id) {
    auto *startId = start_ompl->as<ompl::mrp::RStateSpace::StateType>(id);
    startId->values[0] = starts[id].position_W.x();
    startId->values[1] = starts[id].position_W.y();
    startId->values[2] = starts[id].position_W.z();

    auto *goalId = goal_ompl->as<ompl::mrp::RStateSpace::StateType>(id);
    goalId->values[0] = goals[id].position_W.x();
    goalId->values[1] = goals[id].position_W.y();
    goalId->values[2] = goals[id].position_W.z();
  }

  multi_agent_problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl,
                                                   voxel_size_);
  multi_agent_problem_setup_.setup();
}

void VoxbloxOmplRrt::solutionPathToTrajectoryPoints(
    ompl::geometric::PathGeometric &path,
    mav_msgs::EigenTrajectoryPointVector *trajectory_points) const {
  CHECK_NOTNULL(trajectory_points);
  trajectory_points->clear();
  trajectory_points->reserve(path.getStateCount());

  std::vector<ompl::base::State *> &state_vector = path.getStates();

  for (ompl::base::State *state_ptr : state_vector) {
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[1],
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[2]);

    mav_msgs::EigenTrajectoryPoint mav_trajectory_point;
    mav_trajectory_point.position_W = mav_position;
    trajectory_points->emplace_back(mav_trajectory_point);
  }
}

bool VoxbloxOmplRrt::getBestPathTowardGoal(
    const mav_msgs::EigenTrajectoryPoint &start,
    const mav_msgs::EigenTrajectoryPoint &goal,
    mav_msgs::EigenTrajectoryPoint::Vector *solution) {
  CHECK_NOTNULL(solution);
  solution->clear();
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  bool solution_found = false;
  solution_found = problem_setup_.solve(params_.num_seconds_to_plan);
  if (solution_found) {
    if (problem_setup_.haveSolutionPath()) {
      // Simplify and print.
      if (params_.simplify_solution) {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner) {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
      solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(),
                                     solution);
      return true;
    }
  }
  // The case where you actually have a solution path has returned by now.
  // Otherwise let's just see what the best we can do is.
  ompl::base::PlannerData planner_data(problem_setup_.getSpaceInformation());
  problem_setup_.getPlanner()->getPlannerData(planner_data);

  // Start traversing the graph and find the node that gets the closest to the
  // actual goal point.
  if (planner_data.numStartVertices() < 1) {
    ROS_ERROR("No start vertices in RRT!");
    return false;
  }

  unsigned int min_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  if (planner_data.numVertices() <= 0) {
    ROS_ERROR("No vertices in RRT!");
    return false;
  }

  // Iterate over all vertices. Check which is the closest.
  for (unsigned int i = 0; i < planner_data.numVertices(); i++) {
    const ompl::base::PlannerDataVertex &vertex = planner_data.getVertex(i);
    double distance =
        getDistanceEigenToState(goal.position_W, vertex.getState());

    if (distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  unsigned int start_index = planner_data.getStartIndex(0);

  // Get the closest vertex back out, and then get its parents.
  mav_msgs::EigenTrajectoryPointVector trajectory_points;

  unsigned int current_index = min_index;
  while (current_index != start_index) {
    // Put this vertex in.
    const ompl::base::PlannerDataVertex &vertex =
        planner_data.getVertex(current_index);

    const ompl::base::State *state_ptr = vertex.getState();
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[1],
        state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[2]);

    mav_msgs::EigenTrajectoryPoint mav_trajectory_point;
    mav_trajectory_point.position_W = mav_position;
    trajectory_points.emplace_back(mav_trajectory_point);

    std::vector<unsigned int> edges;

    planner_data.getIncomingEdges(current_index, edges);

    if (edges.empty()) {
      break;
    }

    current_index = edges.front();
  }

  // Finally reverse the vector.
  std::reverse(std::begin(trajectory_points), std::end(trajectory_points));

  *solution = trajectory_points;
  return false;
}

bool VoxbloxOmplRrt::getBestPathTowardGoalMultiAgent(
    const mav_msgs::EigenTrajectoryPointVector &start,
    const mav_msgs::EigenTrajectoryPointVector &goal,
    std::vector<mav_msgs::EigenTrajectoryPoint::Vector> &solutions) {
  uint64_t num_agents = start.size();

  solutions.clear();
  solutions.resize(num_agents);
  setupMultiAgentFromStartAndGoal(start, goal);

  // Solvin' time!
  bool solution_found =
      multi_agent_problem_setup_.solve(params_.num_seconds_to_plan);
  if (solution_found) {
    if (multi_agent_problem_setup_.haveExactSolutionPath()) {
      // Simplify and print.
      if (params_.simplify_solution) {
        multi_agent_problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner) {
        multi_agent_problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }

      std::vector<ompl::base::State *> &state_vector =
          multi_agent_problem_setup_.getSolutionPath().getStates();
      for (ompl::base::State *state_ptr : state_vector) {
        for (size_t id = 0; id < num_agents; ++id) {
          mav_msgs::EigenTrajectoryPoint mav_position;
          mav_position.position_W =
              Eigen::Vector3d(state_ptr->as<ompl::base::CompoundState>()
                                  ->as<ompl::mrp::RStateSpace::StateType>(id)
                                  ->values[0],
                              state_ptr->as<ompl::base::CompoundState>()
                                  ->as<ompl::mrp::RStateSpace::StateType>(id)
                                  ->values[1],
                              state_ptr->as<ompl::base::CompoundState>()
                                  ->as<ompl::mrp::RStateSpace::StateType>(id)
                                  ->values[2]);
          solutions[id].push_back(mav_position);
        }
      }
      return true;
    }
  }

  // In this other case, we don't try to extract anything and just return the
  // failure
  return false;
}

double VoxbloxOmplRrt::getDistanceEigenToState(
    const Eigen::Vector3d &eigen, const ompl::base::State *state_ptr) {
  Eigen::Vector3d state_pos(
      state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[0],
      state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[1],
      state_ptr->as<ompl::mrp::RStateSpace::StateType>()->values[2]);

  return (eigen - state_pos).norm();
}

bool VoxbloxOmplRrt::validStraightLine(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const int n_step,
    std::vector<Eigen::Vector3d> &path) const {
  double delta_x = (goal(0) - start(0)) / static_cast<double>(n_step);
  double delta_y = (goal(1) - start(1)) / static_cast<double>(n_step);
  double delta_z = (goal(2) - start(2)) / static_cast<double>(n_step);

  for (int i = 0; i <= n_step; ++i) {
    Eigen::Vector3d delta_position(delta_x * static_cast<double>(i),
                                   delta_y * static_cast<double>(i),
                                   delta_z * static_cast<double>(i));

    voxblox::Point robot_point =
        (start + delta_position).cast<voxblox::FloatingPoint>();
    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(*tsdf_layer_, robot_point,
                                         robot_radius_, &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
         block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          tsdf_layer_->getBlockPtrByIndex(kv.first);

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

    // If no collision is found, then add the position to the storage
    path.push_back(start + delta_position);
  }

  // Add the final position as well - we already know it is valid
  path.push_back(goal);

  return true;
}

bool VoxbloxOmplRrt::validConstrainedStraightLine(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const int n_step,
    std::vector<std::vector<Eigen::Vector3d> > &con_path,
    std::vector<Eigen::Vector3d> &out_path) const {
  // Try straight line connection first. If this is not feasible already, it
  // does not make sense that we cross check with the paths of the other agents
  if (!validStraightLine(start, goal, n_step, out_path)) {
    return false;
  }

  // Now do the cross - checking
  double min_clearance = params_.safety_factor * robot_radius_;
  for (int id = 0; id < con_path.size(); ++id) {
    for (std::vector<Eigen::Vector3d>::iterator con_it = con_path[id].begin();
         con_it != con_path[id].end(); ++con_it) {
      // Iterate over the found solution
      for (std::vector<Eigen::Vector3d>::iterator out_it = out_path.begin();
           out_it != out_path.end(); ++out_it) {
        if ((*con_it - *out_it).norm() < min_clearance) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace mrp