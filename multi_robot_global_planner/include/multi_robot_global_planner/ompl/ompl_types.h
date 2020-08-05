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