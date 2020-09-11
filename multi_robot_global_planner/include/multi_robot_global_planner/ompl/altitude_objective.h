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
    double z = robot_state(2);  // access the z-coordinate of
    // current state s
    return std::sqrt(std::pow(z - constant_height, 2));
  }

 private:
  double constant_height;
};

}  // end namespace mrp
}  // end namespace ompl
