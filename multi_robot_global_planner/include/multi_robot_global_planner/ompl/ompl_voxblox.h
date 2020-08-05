/*
 * ompl_voxblox.h
 * @brief Header for useful ompl operations with voxblox
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: July 29, 2019
 */

#pragma once

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>

#include "multi_robot_global_planner/ompl/ompl_types.h"

namespace ompl {
namespace mrp {

template <typename VoxelType>
class VoxbloxValidityChecker : public base::StateValidityChecker {
public:
  VoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                         double robot_radius, voxblox::Layer<VoxelType> *layer)
      : base::StateValidityChecker(space_info), layer_(layer),
        robot_radius_(robot_radius) {
    CHECK_NOTNULL(layer);
    voxel_size_ = layer->voxel_size();
  }

  virtual bool isValid(const base::State *state) const {
    Eigen::Vector3d robot_position = omplRToEigen(state);
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    bool collision = checkCollisionWithRobot(robot_position);
    // We check the VALIDITY of the state, and the function above returns
    // whether the state was in COLLISION.
    return !collision;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const = 0;

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const {
    return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
  }

  float voxel_size() const { return voxel_size_; }

protected:
  voxblox::Layer<VoxelType> *layer_;

  float voxel_size_;
  double robot_radius_;
};

template <typename VoxelType>
class VoxbloxValidityCheckerMultiAgentCompoundState :
        public base::StateValidityChecker {
public:
  VoxbloxValidityCheckerMultiAgentCompoundState(
          const base::SpaceInformationPtr &space_info,
          double robot_radius, double safety_factor, const int num_agents,
          voxblox::Layer<VoxelType> *layer)
          : base::StateValidityChecker(space_info), layer_(layer),
            robot_radius_(robot_radius), safety_factor_(safety_factor),
            num_agents_(num_agents) {
    CHECK_NOTNULL(layer);
    voxel_size_ = layer->voxel_size();
  }

  virtual bool isValid(const base::State *state) const {

    // Do checking of the single states
    std::vector<Eigen::Vector3d> positions(num_agents_);
    for(int id = 0; id < num_agents_; ++id) {
      Eigen::Vector3d robot_position(Eigen::Vector3d(
              state->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[0],
              state->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[1],
              state->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[2]));

      if (!si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()
              ->getSubspace(id)->satisfiesBounds(
                  state->as<ompl::base::CompoundState>()
                      ->as<ompl::mrp::RStateSpace::StateType>(id))) {
        return false;
      }
      if(checkCollisionWithRobot(robot_position)) {
        return false;
      }

      // Store the position of the current agent to perform cross-checks
      positions[id] = robot_position;
    }

    // Do the cross-check for robots. Check all combinations
    for(int i = 0; i < num_agents_; ++i) {
      Eigen::Vector3d position_i(positions[i]);
      for(int j = 0; j < num_agents_; ++j) {
        if(j == i) continue;
        Eigen::Vector3d position_j(positions[j]);
        if((position_i - position_j).norm() < robot_radius_) {
          return false;
        }
      }
    }
    return true;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const = 0;

  virtual bool checkCollisionWithRobotAtVoxel(
          const voxblox::GlobalIndex &global_index) const {
    return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
  }

  float voxel_size() const { return voxel_size_; }
  int num_agents() const { return num_agents_; }
  double robot_radius() const { return robot_radius_; }
  double safety_distance() const { return safety_factor_ * robot_radius_; }

protected:
    voxblox::Layer<VoxelType> *layer_;

    float voxel_size_;
    double robot_radius_;
    double safety_factor_;
    int num_agents_;
};

template <typename VoxelType>
class VoxbloxValidityCheckerHierarchicalMultiAgent :
        public base::StateValidityChecker {
public:
    VoxbloxValidityCheckerHierarchicalMultiAgent(
            const base::SpaceInformationPtr &space_info,
            double robot_radius, double safety_factor,
            std::vector<std::vector<Eigen::Vector3d> >& check_paths,
            voxblox::Layer<VoxelType> *layer)
            : base::StateValidityChecker(space_info), layer_(layer),
              robot_radius_(robot_radius), safety_factor_(safety_factor),
              check_paths_(check_paths) {
      CHECK_NOTNULL(layer);
      voxel_size_ = layer->voxel_size();
    }

    virtual bool isValid(const base::State *state) const {

      // Check bounds
      if (!si_->satisfiesBounds(state)) {
        return false;
      }

      // Check the single sampled state against all the other agent's states
      Eigen::Vector3d sample = omplRToEigen(state);

      // If we have only one agent, then we don't have to do checks, except
      // for the map checks
      if(checkCollisionWithRobot(sample)) {
        return false;
      }

      // Here we start checking for the paths in the hierarchy
      for(int id = 1; id < check_paths_.size(); ++id) {
        std::vector<Eigen::Vector3d> path_master = check_paths_[id];
        for(std::vector<Eigen::Vector3d>::iterator it =
                path_master.begin(); it < path_master.end(); ++it) {
          if((sample - (*it)).norm() < safety_factor_ * robot_radius_) {
            return false;
          }
        }
      }

      return true;
    }

    // Returns whether there is a collision: true if yes, false if not.
    virtual bool
    checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const = 0;

    virtual bool checkCollisionWithRobotAtVoxel(
            const voxblox::GlobalIndex &global_index) const {
      return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
    }

    float voxel_size() const { return voxel_size_; }
    double robot_radius() const { return robot_radius_; }
    double safety_distance() const { return safety_factor_ * robot_radius_; }
    std::vector<std::vector<Eigen::Vector3d> > check_paths() const {
      return check_paths_;
    }

protected:
    voxblox::Layer<VoxelType> *layer_;

    float voxel_size_;
    double robot_radius_;
    double safety_factor_;
    std::vector<std::vector<Eigen::Vector3d> > check_paths_;
};

class TsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::TsdfVoxel> {
public:
  TsdfVoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                             double robot_radius,
                             voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, tsdf_layer),
        treat_unknown_as_occupied_(false) {}

  bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
  void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
    treat_unknown_as_occupied_ = treat_unknown_as_occupied;
  }

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                         &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
         block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex &voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        const voxblox::TsdfVoxel &tsdf_voxel =
            block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.weight < voxblox::kEpsilon) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        if (tsdf_voxel.distance <= 0.0f) {
          return true;
        }
      }
    }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
    return false;
  }

protected:
  bool treat_unknown_as_occupied_;
};

class TsdfVoxbloxValidityCheckerMultiAgentCompoundState
        : public VoxbloxValidityCheckerMultiAgentCompoundState<
                voxblox::TsdfVoxel> {
public:
    TsdfVoxbloxValidityCheckerMultiAgentCompoundState(
            const base::SpaceInformationPtr &space_info,
            double robot_radius, double safety_factor,
            const int num_agents,
            voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer)
            : VoxbloxValidityCheckerMultiAgentCompoundState(
                    space_info, robot_radius, safety_factor,
                    num_agents, tsdf_layer),
              treat_unknown_as_occupied_(false) {}

    bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
    void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
      treat_unknown_as_occupied_ = treat_unknown_as_occupied;
    }

    virtual bool
    checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
      voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

      voxblox::HierarchicalIndexMap block_voxel_list;
      voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                           &block_voxel_list);

      for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
              block_voxel_list) {
        // Get block -- only already existing blocks are in the list.
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
                layer_->getBlockPtrByIndex(kv.first);

        if (!block_ptr) {
          continue;
        }

        for (const voxblox::VoxelIndex &voxel_index : kv.second) {
          if (!block_ptr->isValidVoxelIndex(voxel_index)) {
            if (treat_unknown_as_occupied_) {
              return true;
            }
            continue;
          }
          const voxblox::TsdfVoxel &tsdf_voxel =
                  block_ptr->getVoxelByVoxelIndex(voxel_index);
          if (tsdf_voxel.weight < voxblox::kEpsilon) {
            if (treat_unknown_as_occupied_) {
              return true;
            }
            continue;
          }
          if (tsdf_voxel.distance <= 0.0f) {
            return true;
          }
        }
      }

      // No collision if nothing in the sphere had a negative or 0 distance.
      // Unknown space is unoccupied, since this is a very optimistic global
      // planner.
      return false;
    }

protected:
    bool treat_unknown_as_occupied_;
};

class TsdfVoxbloxValidityCheckerHierarchicalMultiAgent
        : public VoxbloxValidityCheckerHierarchicalMultiAgent<
                voxblox::TsdfVoxel> {
public:
    TsdfVoxbloxValidityCheckerHierarchicalMultiAgent(
            const base::SpaceInformationPtr &space_info,
            double robot_radius, double safety_factor,
            std::vector<std::vector<Eigen::Vector3d> > &check_paths,
            voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer)
            : VoxbloxValidityCheckerHierarchicalMultiAgent(
                  space_info, robot_radius, safety_factor,
                  check_paths, tsdf_layer),
              treat_unknown_as_occupied_(false) {}

    bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
    void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
      treat_unknown_as_occupied_ = treat_unknown_as_occupied;
    }

    virtual bool
    checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
      voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

      voxblox::HierarchicalIndexMap block_voxel_list;
      voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                           &block_voxel_list);

      for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
              block_voxel_list) {
        // Get block -- only already existing blocks are in the list.
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
                layer_->getBlockPtrByIndex(kv.first);

        if (!block_ptr) {
          continue;
        }

        for (const voxblox::VoxelIndex &voxel_index : kv.second) {
          if (!block_ptr->isValidVoxelIndex(voxel_index)) {
            if (treat_unknown_as_occupied_) {
              return true;
            }
            continue;
          }
          const voxblox::TsdfVoxel &tsdf_voxel =
                  block_ptr->getVoxelByVoxelIndex(voxel_index);
          if (tsdf_voxel.weight < voxblox::kEpsilon) {
            if (treat_unknown_as_occupied_) {
              return true;
            }
            continue;
          }
          if (tsdf_voxel.distance <= 0.0f) {
            return true;
          }
        }
      }

      // No collision if nothing in the sphere had a negative or 0 distance.
      // Unknown space is unoccupied, since this is a very optimistic global
      // planner.
      return false;
    }

protected:
    bool treat_unknown_as_occupied_;
};

class EsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::EsdfVoxel> {
public:
  EsdfVoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                             double robot_radius,
                             voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, esdf_layer),
        interpolator_(esdf_layer) {}

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
    constexpr bool interpolate = false;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(
        robot_position.cast<voxblox::FloatingPoint>(), &distance, interpolate);
    if (!success) {
      return true;
    }

    return robot_radius_ >= distance;
  }

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const {
    voxblox::EsdfVoxel *voxel = layer_->getVoxelPtrByGlobalIndex(global_index);

    if (voxel == nullptr) {
      return true;
    }
    return robot_radius_ >= voxel->distance;
  }

protected:
  // Interpolator for the layer.
  voxblox::Interpolator<voxblox::EsdfVoxel> interpolator_;
};

class EsdfVoxbloxValidityCheckerHierarchicalMultiAgent
        : public VoxbloxValidityCheckerHierarchicalMultiAgent<
                voxblox::EsdfVoxel> {
public:
  EsdfVoxbloxValidityCheckerHierarchicalMultiAgent(
          const base::SpaceInformationPtr &space_info,
          double robot_radius,
          double safety_factor,
          std::vector<std::vector<Eigen::Vector3d> > &check_paths,
          voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer)
          : VoxbloxValidityCheckerHierarchicalMultiAgent(
          space_info, robot_radius, safety_factor, check_paths, esdf_layer),
            interpolator_(esdf_layer) {}

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
    constexpr bool interpolate = false;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(
            robot_position.cast<voxblox::FloatingPoint>(), &distance,
            interpolate);
    if (!success) {
      return true;
    }

    return robot_radius_ >= distance;
  }

  virtual bool checkCollisionWithRobotAtVoxel(
          const voxblox::GlobalIndex &global_index) const {
    voxblox::EsdfVoxel *voxel = layer_->getVoxelPtrByGlobalIndex(global_index);

    if (voxel == nullptr) {
      return true;
    }
    return robot_radius_ >= voxel->distance;
  }

protected:
    // Interpolator for the layer.
    voxblox::Interpolator<voxblox::EsdfVoxel> interpolator_;
};

// Motion validator that uses either of the validity checkers above to
// validate motions at voxel resolution.
template <typename VoxelType>
class VoxbloxMotionValidator : public base::MotionValidator {
public:
  VoxbloxMotionValidator(
      const base::SpaceInformationPtr &space_info,
      typename std::shared_ptr<VoxbloxValidityChecker<VoxelType>>
          validity_checker)
      : base::MotionValidator(space_info), validity_checker_(validity_checker) {
    CHECK(validity_checker);
  }

  virtual bool checkMotion(const base::State *s1, const base::State *s2) const {
    std::pair<base::State *, double> unused;
    return checkMotion(s1, s2, unused);
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So opposite of checkCollision, but same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  virtual bool checkMotion(const base::State *s1, const base::State *s2,
                           std::pair<base::State *, double> &last_valid) const {
    Eigen::Vector3d start = omplRToEigen(s1);
    Eigen::Vector3d goal = omplRToEigen(s2);
    double voxel_size = validity_checker_->voxel_size();

    voxblox::Point start_scaled, goal_scaled;
    voxblox::AlignedVector<voxblox::GlobalIndex> indices;

    // Convert the start and goal to global voxel coordinates.
    // Actually very simple -- just divide by voxel size.
    start_scaled = start.cast<voxblox::FloatingPoint>() / voxel_size;
    goal_scaled = goal.cast<voxblox::FloatingPoint>() / voxel_size;

    voxblox::castRay(start_scaled, goal_scaled, &indices);

    for (size_t i = 0; i < indices.size(); i++) {
      const voxblox::GlobalIndex &global_index = indices[i];

      Eigen::Vector3d pos = global_index.cast<double>() * voxel_size;
      bool collision =
          validity_checker_->checkCollisionWithRobotAtVoxel(global_index);

      if (collision) {
        if (last_valid.first != nullptr) {
          ompl::base::ScopedState<ompl::mrp::RStateSpace> last_valid_state(
              si_->getStateSpace());
          last_valid_state->values[0] = pos.x();
          last_valid_state->values[1] = pos.y();
          last_valid_state->values[2] = pos.z();

          si_->copyState(last_valid.first, last_valid_state.get());
        }

        last_valid.second = static_cast<double>(i / indices.size());
        return false;
      }
    }

    return true;
  }

protected:
  typename std::shared_ptr<VoxbloxValidityChecker<VoxelType>> validity_checker_;
};

template <typename VoxelType>
class VoxbloxMotionValidatorMultiAgentCompoundState :
        public base::MotionValidator {
public:
    VoxbloxMotionValidatorMultiAgentCompoundState(
            const base::SpaceInformationPtr &space_info,
            typename std::shared_ptr<
                    VoxbloxValidityCheckerMultiAgentCompoundState<VoxelType>>
            validity_checker)
            : base::MotionValidator(space_info), validity_checker_(validity_checker) {
      CHECK(validity_checker);
    }

    virtual bool checkMotion(const base::State *s1, const base::State *s2) const {
      std::pair<base::State *, double> unused;
      return checkMotion(s1, s2, unused);
    }

    // Check motion returns *false* if invalid, *true* if valid.
    // So opposite of checkCollision, but same as isValid.
    // last_valid is the state and percentage along the trajectory that's
    // a valid state.
    virtual bool checkMotion(const base::State *s1,
            const base::State *s2, std::pair<base::State *,
            double> &last_valid) const {

      Eigen::Vector3d start, goal;
      std::vector<std::vector<Eigen::Vector3d> > points(
              validity_checker_->num_agents());

      for(int id = 0; id < validity_checker_->num_agents(); ++id) {
        start << s1->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[0],
                 s1->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[1],
                 s1->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[2];
        goal  << s2->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[0],
                 s2->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[1],
                 s2->as<ompl::base::CompoundState>()
                   ->as<ompl::mrp::RStateSpace::StateType>(id)->values[2];

        // Store the points for successive checks
        points[id].push_back(start);
        points[id].push_back(goal);

        double voxel_size = validity_checker_->voxel_size();
        voxblox::Point start_scaled, goal_scaled;
        voxblox::AlignedVector<voxblox::GlobalIndex> indices;

        // Convert the start and goal to global voxel coordinates.
        // Actually very simple -- just divide by voxel size.
        start_scaled = start.cast<voxblox::FloatingPoint>() / voxel_size;
        goal_scaled = goal.cast<voxblox::FloatingPoint>() / voxel_size;

        voxblox::castRay(start_scaled, goal_scaled, &indices);

        for (size_t i = 0; i < indices.size(); i++) {
          const voxblox::GlobalIndex &global_index = indices[i];

          Eigen::Vector3d pos = global_index.cast<double>() * voxel_size;
          points[id].push_back(pos);
          bool collision =
                  validity_checker_->checkCollisionWithRobotAtVoxel(
                          global_index);

          if(collision) { return false; }

          // Check for cross intersections
          if (id != 0) {
            for(int j = 0; j < id; ++j) {
              if(id == j) continue;

              for(std::vector<Eigen::Vector3d>::iterator it = points[j].begin();
                  it < points[j].end(); ++it) {
                if(((*it) - pos).norm() <
                       validity_checker_->safety_distance()) {
                  return false;
                }
              }
            }
          }
        }
      }
      return true;
    }

protected:
    typename std::shared_ptr<VoxbloxValidityCheckerMultiAgentCompoundState<
            VoxelType>> validity_checker_;
};

template <typename VoxelType>
class VoxbloxMotionValidatorHierarchicalMultiAgent :
        public base::MotionValidator {
public:
    VoxbloxMotionValidatorHierarchicalMultiAgent(
            const base::SpaceInformationPtr &space_info,
            typename std::shared_ptr<
                    VoxbloxValidityCheckerHierarchicalMultiAgent<VoxelType>>
            validity_checker)
            : base::MotionValidator(space_info),
              validity_checker_(validity_checker) {
      CHECK(validity_checker);
    }

    virtual bool checkMotion(const base::State *s1, const base::State *s2) const {
      std::pair<base::State *, double> unused;
      return checkMotion(s1, s2, unused);
    }


    bool checkSelfMotion(const base::State *s1, const base::State *s2,
                         std::pair<base::State *, double> &last_valid) const {
      Eigen::Vector3d start = omplRToEigen(s1);
      Eigen::Vector3d goal = omplRToEigen(s2);
      double voxel_size = validity_checker_->voxel_size();

      voxblox::Point start_scaled, goal_scaled;
      voxblox::AlignedVector<voxblox::GlobalIndex> indices;

      // Convert the start and goal to global voxel coordinates.
      // Actually very simple -- just divide by voxel size.
      start_scaled = start.cast<voxblox::FloatingPoint>() / voxel_size;
      goal_scaled = goal.cast<voxblox::FloatingPoint>() / voxel_size;

      voxblox::castRay(start_scaled, goal_scaled, &indices);

      for (size_t i = 0; i < indices.size(); i++) {
        const voxblox::GlobalIndex &global_index = indices[i];

        Eigen::Vector3d pos = global_index.cast<double>() * voxel_size;
        bool collision =
                validity_checker_->checkCollisionWithRobotAtVoxel(global_index);

        if (collision) {
          if (last_valid.first != nullptr) {
            ompl::base::ScopedState<ompl::mrp::RStateSpace> last_valid_state(
                    si_->getStateSpace());
            last_valid_state->values[0] = pos.x();
            last_valid_state->values[1] = pos.y();
            last_valid_state->values[2] = pos.z();

            si_->copyState(last_valid.first, last_valid_state.get());
          }

          last_valid.second = static_cast<double>(i / indices.size());
          return false;
        }
      }

      return true;
    }

    bool checkSelfMotion(const base::State *s1, const base::State *s2) const {
      std::pair<base::State *, double> unused;
      return checkSelfMotion(s1, s2, unused);
    }

    // Check motion returns *false* if invalid, *true* if valid.
    // So opposite of checkCollision, but same as isValid.
    // last_valid is the state and percentage along the trajectory that's
    // a valid state.
    virtual bool checkMotion(const base::State *s1,
                             const base::State *s2,
                             std::pair<base::State *, double> &last_valid)
                             const {

      // First, check self motion
      if(!checkSelfMotion(s1, s2)) {
        return false;
      }

      // Now check against the paths of the other agents. Divide the segment
      // connecting s1 and s2 every 2*robot radius meters. Then check all
      // these positons
      Eigen::Vector3d start = omplRToEigen(s1);
      Eigen::Vector3d goal = omplRToEigen(s2);
      Eigen::Vector3d direction( (goal - start).normalized() );

      int steps = std::floor((goal-start).norm() /
              validity_checker_->safety_distance());
      std::vector<std::vector<Eigen::Vector3d> > check_paths(
              validity_checker_->check_paths());

      for(int i = 0; i < steps; ++i) {
        Eigen::Vector3d point = start + static_cast<double>(i) /
                static_cast<double>(steps) * (start - goal).norm() * direction;

        for(int k = 0; k < check_paths.size(); ++k) {
          if(!checkDistancePointPath(point, check_paths[k])) {
            return false;
          }
        }
      }

      return true;
    }

    bool checkDistancePointPath(const Eigen::Vector3d &point,
            const std::vector<Eigen::Vector3d> &path) const {
      for(auto it = path.begin(); it < path.end(); ++it) {
        if((point - (*it)).norm() < validity_checker_->safety_distance()) {
          return false;
        }
      }
      return true;
    }

protected:
    typename std::shared_ptr<VoxbloxValidityCheckerHierarchicalMultiAgent<
            VoxelType>> validity_checker_;
};

} // namespace mrp
} // namespace ompl