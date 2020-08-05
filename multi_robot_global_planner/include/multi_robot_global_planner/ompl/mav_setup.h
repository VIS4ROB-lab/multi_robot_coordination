/*
 * mav_setup.h
 * @brief Header for useful ompl operations with voxblox for the robot
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: July 29, 2019
 */

#pragma once

#include <functional>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "multi_robot_global_planner/ompl/altitude_objective.h"
#include "multi_robot_global_planner/ompl/multi_agent_sampler.h"

namespace ompl {
namespace mrp {

// Setup class for a geometric planning problem with R3 state space.
class MavSetup : public geometric::SimpleSetup {
public:
  MavSetup()
      : geometric::SimpleSetup(base::StateSpacePtr(new RStateSpace(3))) {}

  // Get some defaults.
  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(
            new ompl::base::PathLengthOptimizationObjective(
                getSpaceInformation())));
  }

  void setConstantAltitudeObjective(const double h, double alpha, double beta) {
    // OMPL: provide optimisation objective to problem definition by passing an
    // OptimizationObjectivePtr from getBalancedObjective(,,,)
    getProblemDefinition()->setOptimizationObjective(
        getBalancedObjective(h, alpha, beta));
  }

  // Returns OptimizationObjectivePtr for weighted multi-objective problem
  // consiting of path length and altitude
  // Params w1, w2 : given by alpha, beta in launch file
  // Param h: altitude/height of MAV when planning begins (via service call)
  ompl::base::OptimizationObjectivePtr
  getBalancedObjective(const double h, const double w1, const double w2) {
    ompl::base::OptimizationObjectivePtr lengthObj(
        new ompl::base::PathLengthOptimizationObjective(getSpaceInformation()));
    ompl::base::OptimizationObjectivePtr altitudeObj(
        new ompl::mrp::AltitudeObjective(getSpaceInformation(), h));

    return w1 * lengthObj + w2 * altitudeObj;
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTstar(getSpaceInformation())));
  }

  void setRrtConnect() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTConnect(getSpaceInformation())));
  }

  void setInformedRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::InformedRRTstar(getSpaceInformation())));
  }

  void setBitStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::BITstar(getSpaceInformation())));
  }

  void setPrm() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::PRM(getSpaceInformation())));
  }

  const base::StateSpacePtr &getGeometricComponentStateSpace() const {
    return getStateSpace();
  }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setLenghtOptimizationObjective(const double distance) {
    ompl::base::OptimizationObjectivePtr obj(
            new ompl::base::PathLengthOptimizationObjective(
                    getSpaceInformation()));
    obj->setCostThreshold(ompl::base::Cost(distance));
    getProblemDefinition()->setOptimizationObjective(obj);
  }

  void setTsdfVoxbloxCollisionChecking(
      double robot_radius, voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {
    std::shared_ptr<TsdfVoxbloxValidityChecker> validity_checker(
        new TsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       tsdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::TsdfVoxel>(
            getSpaceInformation(), validity_checker)));
  }

  void setTsdfVoxbloxCollisionCheckingHierarchical(
          double robot_radius, double safety_factor,
          std::vector<std::vector<Eigen::Vector3d> > &check_paths,
          voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {

    std::shared_ptr<TsdfVoxbloxValidityCheckerHierarchicalMultiAgent>
            validity_checker(
            new TsdfVoxbloxValidityCheckerHierarchicalMultiAgent(
                    getSpaceInformation(), robot_radius, safety_factor,
                    check_paths, tsdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
            base::MotionValidatorPtr(
                    new VoxbloxMotionValidatorHierarchicalMultiAgent<
                            voxblox::TsdfVoxel>(
                            getSpaceInformation(), validity_checker)));
  }

  void setEsdfVoxbloxCollisionChecking(
      double robot_radius, voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
    std::shared_ptr<EsdfVoxbloxValidityChecker> validity_checker(
        new EsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       esdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::EsdfVoxel>(
            getSpaceInformation(), validity_checker)));
  }

  void setEsdfVoxbloxCollisionCheckingHierarchical(
          double robot_radius, double safety_factor,
          std::vector<std::vector<Eigen::Vector3d> > &check_paths,
          voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
    std::shared_ptr<EsdfVoxbloxValidityCheckerHierarchicalMultiAgent>
            validity_checker(
            new EsdfVoxbloxValidityCheckerHierarchicalMultiAgent(
                    getSpaceInformation(), robot_radius, safety_factor,
                    check_paths, esdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
            base::MotionValidatorPtr(
                    new VoxbloxMotionValidatorHierarchicalMultiAgent<
                            voxblox::EsdfVoxel>(
                            getSpaceInformation(), validity_checker)));
  }

  void constructPrmRoadmap(double num_seconds_to_construct) {
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(num_seconds_to_construct);

    std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
        ->constructRoadmap(ptc);
  }

  static base::ValidStateSamplerPtr allocMultiAgentStateSampler(
          const base::SpaceInformation  *si,
          const double robot_radius, const double safety_factor,
          std::vector<std::vector<Eigen::Vector3d> > &constraint_paths) {
    std::shared_ptr<MultiAgentSampler> sampler(new
                   ompl::mrp::MultiAgentSampler(si, robot_radius,
                           safety_factor, constraint_paths));
    return sampler;
  }

  void setMultiAgentSampler(const double robot_radius,
                const double safety_factor,
                std::vector<std::vector<Eigen::Vector3d> > &constraint_paths) {
    std::function<ompl::base::ValidStateSamplerPtr(
            const ompl::base::SpaceInformation*)> sampler_allocator =
            std::bind(&ompl::mrp::MavSetup::allocMultiAgentStateSampler,
                      std::placeholders::_1, robot_radius, safety_factor,
                      constraint_paths);
    getSpaceInformation()->setValidStateSamplerAllocator(sampler_allocator);
  }

  // Uses the path simplifier WITHOUT using B-spline smoothing which leads to
  // a lot of issues for us.
  void reduceVertices() {
    if (pdef_) {
      const base::PathPtr &p = pdef_->getSolutionPath();
      if (p) {
        time::point start = time::now();
        geometric::PathGeometric &path =
            static_cast<geometric::PathGeometric &>(*p);
        std::size_t num_states = path.getStateCount();

        reduceVerticesOfPath(path);
        // simplifyTime_ member of the parent class.
        simplifyTime_ = time::seconds(time::now() - start);
        OMPL_INFORM(
            "MavSetup: Vertex reduction took %f seconds and changed from %d to "
            "%d states",
            simplifyTime_, num_states, path.getStateCount());
        return;
      }
    }
    OMPL_WARN("No solution to simplify");
  }

  // Simplification of path without B-splines.
  void reduceVerticesOfPath(geometric::PathGeometric &path) {
    const double max_time = 0.1;
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(max_time);

    // Now just call near-vertex collapsing and reduceVertices.
    if (path.getStateCount() < 3) {
      return;
    }

    // try a randomized step of connecting vertices
    bool try_more = false;
    if (ptc == false) {
      try_more = psk_->reduceVertices(path);
    }

    // try to collapse close-by vertices
    if (ptc == false) {
      psk_->collapseCloseVertices(path);
    }

    // try to reduce verices some more, if there is any point in doing so
    int times = 0;
    while (try_more && ptc == false && ++times <= 5) {
      try_more = psk_->reduceVertices(path);
    }
  }
}; // end class MavSetup

/**
 * @brief Class for setting up multi-robot problem
 */
class MultiMavSetup : public geometric::SimpleSetup {
public:
  MultiMavSetup(const int num_agents = 1)
    : geometric::SimpleSetup(base::StateSpacePtr(
            new base::CompoundStateSpace())),
      num_agents_(num_agents) {
    for (unsigned int i = 0; i < num_agents; ++i) {
      si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->addSubspace(
              std::make_shared<ompl::mrp::RStateSpace>(3), 1.0);
    }
  }

  void setNumAgents(const int num_agents) { num_agents_ = num_agents; }

  // Get some defaults.
  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
            ompl::base::OptimizationObjectivePtr(
                    new ompl::base::PathLengthOptimizationObjective(
                            getSpaceInformation())));
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
            new ompl::geometric::RRTstar(getSpaceInformation())));
  }

  void setRrtConnect() {
    setPlanner(ompl::base::PlannerPtr(
            new ompl::geometric::RRTConnect(getSpaceInformation())));
  }

  void setInformedRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
            new ompl::geometric::InformedRRTstar(getSpaceInformation())));
  }

  void setBitStar() {
    setPlanner(ompl::base::PlannerPtr(
            new ompl::geometric::BITstar(getSpaceInformation())));
  }

  void setPrm() {
    setPlanner(ompl::base::PlannerPtr(
            new ompl::geometric::PRM(getSpaceInformation())));
  }

  const base::StateSpacePtr &getGeometricComponentStateSpace() const {
    return getStateSpace();
  }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setLenghtOptimizationObjective(const double distance) {
    ompl::base::OptimizationObjectivePtr obj(
            new ompl::base::PathLengthOptimizationObjective(
                    getSpaceInformation()));
    obj->setCostThreshold(ompl::base::Cost(distance));
    getProblemDefinition()->setOptimizationObjective(obj);
  }

  void setTsdfVoxbloxCollisionCheckingCompoundState(
          double robot_radius, double safety_factor,
          voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {

    std::shared_ptr<TsdfVoxbloxValidityCheckerMultiAgentCompoundState>
            validity_checker(
            new TsdfVoxbloxValidityCheckerMultiAgentCompoundState(
                    getSpaceInformation(), robot_radius, safety_factor,
                    num_agents_, tsdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
            base::MotionValidatorPtr(
                    new VoxbloxMotionValidatorMultiAgentCompoundState<
                            voxblox::TsdfVoxel>(
                            getSpaceInformation(), validity_checker)));
  }

  void setEsdfVoxbloxCollisionChecking(
          double robot_radius, voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
    std::shared_ptr<EsdfVoxbloxValidityChecker> validity_checker(
            new EsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                           esdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
            base::MotionValidatorPtr(
                    new VoxbloxMotionValidator<voxblox::EsdfVoxel>(
                    getSpaceInformation(), validity_checker)));
  }

  void constructPrmRoadmap(double num_seconds_to_construct) {
    base::PlannerTerminationCondition ptc =
            base::timedPlannerTerminationCondition(num_seconds_to_construct);

    std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
            ->constructRoadmap(ptc);
  }

  void reduceVertices() {
    if (pdef_) {
      const base::PathPtr &p = pdef_->getSolutionPath();
      if (p) {
        time::point start = time::now();
        geometric::PathGeometric &path =
                static_cast<geometric::PathGeometric &>(*p);
        std::size_t num_states = path.getStateCount();

        reduceVerticesOfPath(path);
        // simplifyTime_ member of the parent class.
        simplifyTime_ = time::seconds(time::now() - start);
        OMPL_INFORM(
                "MavSetup: Vertex reduction took %f seconds and changed from %d to "
                "%d states",
                simplifyTime_, num_states, path.getStateCount());
        return;
      }
    }
    OMPL_WARN("No solution to simplify");
  }

  // Simplification of path without B-splines.
  void reduceVerticesOfPath(geometric::PathGeometric &path) {
    const double max_time = 0.1;
    base::PlannerTerminationCondition ptc =
            base::timedPlannerTerminationCondition(max_time);

    // Now just call near-vertex collapsing and reduceVertices.
    if (path.getStateCount() < 3) {
      return;
    }

    // try a randomized step of connecting vertices
    bool try_more = false;
    if (ptc == false) {
      try_more = psk_->reduceVertices(path);
    }

    // try to collapse close-by vertices
    if (ptc == false) {
      psk_->collapseCloseVertices(path);
    }

    // try to reduce verices some more, if there is any point in doing so
    int times = 0;
    while (try_more && ptc == false && ++times <= 5) {
      try_more = psk_->reduceVertices(path);
    }
  }

protected:
  int num_agents_;
}; // end class MultiMavSetup

} // namespace mrp
} // namespace ompl