/*
 * ompl_voxblox_rrt.h
 * @brief Header for implementation for ompl rrt using voxblox
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: July 29, 2019
 */

#pragma once

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include "multi_robot_global_planner/ompl/mav_setup.h"
#include "multi_robot_global_planner/interpolators/polynomial_interpolator.h"

namespace mrp {

class VoxbloxOmplRrt {
public:
  VoxbloxOmplRrt(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  VoxbloxOmplRrt(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 const int num_agents);
  virtual ~VoxbloxOmplRrt() {}

  inline void setRobotRadius(double robot_radius) {
    robot_radius_ = robot_radius;
  }
  void setBounds(const Eigen::Vector3d &lower_bound,
                 const Eigen::Vector3d &upper_bound);

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer);
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer);

  inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }
  bool getOptimistic() const { return optimistic_; }

  double getNumSecondsToPlan() const { return params_.num_seconds_to_plan; }
  void setNumSecondsToPlan(double num_seconds) {
    params_.num_seconds_to_plan = num_seconds;
  }

  RrtPlannerType getPlanner() const { return params_.planner_type; }
  void setPlanner(RrtPlannerType planner) { params_.planner_type = planner; }

  void setConstraintPaths(const
              std::vector<std::vector<Eigen::Vector3d> >& constraint_paths);

  // Only call this once, only call this after setting all settings correctly.
  void setupMultiAgentProblem(const int num_agents);
  void setupProblem(const Eigen::Vector3d &start,
                    const Eigen::Vector3d &goal,
                    const bool constrained_planning = false);
  void setupReturnHomeProblem(const Eigen::Vector3d &start,
                    const Eigen::Vector3d &goal,
                    const bool constrained_planning = false);

  // Fixed start and end locations, returns list of waypoints between.
  bool
  getPathBetweenWaypoints(const mav_msgs::EigenTrajectoryPoint &start,
                          const mav_msgs::EigenTrajectoryPoint &goal,
                          mav_msgs::EigenTrajectoryPoint::Vector *solution);

  bool getBestPathTowardGoalMultiAgent(
          const mav_msgs::EigenTrajectoryPointVector &start,
          const mav_msgs::EigenTrajectoryPointVector &goal,
          std::vector<mav_msgs::EigenTrajectoryPoint::Vector> &solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric &path,
      mav_msgs::EigenTrajectoryPointVector *trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const mav_msgs::EigenTrajectoryPoint &start,
                             const mav_msgs::EigenTrajectoryPoint &goal,
                             mav_msgs::EigenTrajectoryPoint::Vector *solution);

  void constructPrmRoadmap(double roadmap_construction_sec) {
    problem_setup_.setup();
    problem_setup_.constructPrmRoadmap(roadmap_construction_sec);
  }

  GlobalPlannerParams getParameters() const { return params_; }
  void setParameters(const GlobalPlannerParams &params) { params_ = params; }

  bool validStraightLine(const Eigen::Vector3d &start,
                         const Eigen::Vector3d &goal, const int n_step,
                         std::vector<Eigen::Vector3d> &path) const;

  bool validConstrainedStraightLine(
            const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
            const int n_step, std::vector<std::vector<Eigen::Vector3d> > &con_path,
            std::vector<Eigen::Vector3d> &out_path) const;

protected:
  void setupFromStartAndGoal(const mav_msgs::EigenTrajectoryPoint &start,
                             const mav_msgs::EigenTrajectoryPoint &goal);

  void setupMultiAgentFromStartAndGoal(
          const mav_msgs::EigenTrajectoryPointVector &starts,
          const mav_msgs::EigenTrajectoryPointVector &goals);

  double getDistanceEigenToState(const Eigen::Vector3d &eigen,
                                 const ompl::base::State *state_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Setup the problem in OMPL.
  ompl::mrp::MavSetup problem_setup_;
  ompl::mrp::MultiMavSetup multi_agent_problem_setup_;
  GlobalPlannerParams params_;
  std::vector<std::vector<Eigen::Vector3d> > constraint_paths_;

  // Whether the planner is optimistic (true) or pessimistic (false) about
  // how unknown space is handled.
  // Optimistic uses the TSDF for collision checking, while pessimistic uses
  // the ESDF. Be sure to set the maps accordingly.
  bool optimistic_;

  // Planning bounds, if set.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  // NON-OWNED pointers to the relevant layers. TSDF only used if optimistic,
  // ESDF only used if pessimistic.
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;

  double voxel_size_;
  double robot_radius_;
};

} // namespace mrp