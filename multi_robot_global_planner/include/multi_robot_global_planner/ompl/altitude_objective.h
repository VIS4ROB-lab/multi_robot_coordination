/*
 * altitude_objective.h
 * @brief Header for ompl objective to plan at constant altitude
 * @author: Emilk Sempertegui
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: 01.08.2019
 */

#pragma once

#include <cmath>

#include <ompl/base/Cost.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "multi_robot_global_planner/ompl/ompl_types.h"

namespace ompl {
namespace mrp {

class AltitudeObjective : public ompl::base::StateCostIntegralObjective {
public:
  AltitudeObjective(const ompl::base::SpaceInformationPtr &si, const double h)
      : constant_height(h), ompl::base::StateCostIntegralObjective(si, true) {}

  ompl::base::Cost stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(this->altitude_deviation(s));
  }

  double altitude_deviation(const ompl::base::State *s) const {
    const Eigen::Vector3d robot_state = omplRToEigen(s);
    double z = robot_state(2); // access the z-coordinate of
    // current state s
    return std::sqrt(std::pow(z - constant_height, 2));
  }

private:
  double constant_height;
};

} // end namespace mrp
} // end namespace ompl
