/*
 * history_sampler.h
 * @brief Header for new ompl sampler for multiple agents
 * @author: Luca Bartolomei, V4RL
 * @date  : 08.08.2019
 */

#pragma once

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <Eigen/Eigen>

#include "multi_robot_global_planner/ompl/ompl_voxblox.h"

namespace ompl {
namespace mrp {

class MultiAgentSampler : public base::UniformValidStateSampler {
 public:
  MultiAgentSampler(
      const base::SpaceInformation *si, const double robot_radius,
      const double safety_factor,
      std::vector<std::vector<Eigen::Vector3d> > &constraint_paths)
      : base::UniformValidStateSampler(si),
        robot_radius_(robot_radius),
        safety_factor_(safety_factor) {
    name_ = "Multi Agent Sampler";
    constraint_paths_ = constraint_paths;
  }

  void setRobotRadius(const double robot_radius) {
    robot_radius_ = robot_radius;
  }

  bool sample(base::State *state) override {
    unsigned int attempts = 0;
    bool valid = false;
    do {
      sampler_->sampleUniform(state);
      valid = si_->isValid(state);
      ++attempts;
    } while (!valid && attempts < attempts_);

    if (!valid) return false;

    // Now check if the sample is too close to an existing path
    Eigen::Vector3d sample = omplRToEigen(state);
    for (int id = 0; id < constraint_paths_.size(); ++id) {
      // Iterate over the waypoints of the current agent
      for (std::vector<Eigen::Vector3d>::iterator it =
               constraint_paths_[id].begin();
           it < constraint_paths_[id].end(); ++it) {
        if ((sample - *it).norm() < safety_factor_ * robot_radius_) {
          return false;
        }
      }
    }
    return true;
  }

 protected:
  double robot_radius_;
  double safety_factor_;
  std::vector<std::vector<Eigen::Vector3d> > constraint_paths_;

};  // end class MultiAgentSampler

}  // end namespace mrp
}  // end namespace ompl
