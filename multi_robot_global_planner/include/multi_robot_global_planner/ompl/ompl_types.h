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
 * ompl_typed.h
 * @brief Header for useful transformation with ompl and eigen
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: July 29, 2019
 */

#pragma once

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <Eigen/Core>

namespace ompl {
namespace mrp {

// Take care of the position only
typedef base::RealVectorStateSpace RStateSpace;
typedef base::SE2StateSpace SE2StateSpace;

inline Eigen::Vector3d omplRToEigen(const base::State *state) {
  const mrp::RStateSpace::StateType *derived =
      static_cast<const mrp::RStateSpace::StateType *>(state);
  Eigen::Vector3d eigen_state(derived->values[0], derived->values[1],
                              derived->values[2]);

  return eigen_state;
}

inline Eigen::Vector3d omplSE2ToEigen(const base::State *state) {
  const mrp::SE2StateSpace::StateType *derived =
      static_cast<const mrp::SE2StateSpace::StateType *>(state);
  Eigen::Vector3d eigen_state(derived->getX(), derived->getY(),
                              derived->getYaw());

  return eigen_state;
}

}  // end namespace mrp
}  // end namespace ompl
