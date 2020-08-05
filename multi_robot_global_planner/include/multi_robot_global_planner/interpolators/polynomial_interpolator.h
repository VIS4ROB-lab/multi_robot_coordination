/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for polynomial interpolator
 * @date   01.08.2019
 */

#pragma once

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "multi_robot_global_planner/interpolators/ramp_interpolator.h"

namespace mrp {

class PolynomialInterpolator {
public:
  PolynomialInterpolator(const DynamicParams &params);
  virtual ~PolynomialInterpolator() {}

  bool interpolate(const GlobalPath &path, GlobalPath *interpolated_path);

  void updateParameters(const double v_max, const double a_max,
                        const double v_yaw_max, const double a_yaw_max,
                        const double sampling_dt);

private:
  void
  createYawsFromStates(const mav_msgs::EigenTrajectoryPoint::Vector &states,
                       std::vector<double> &yaws);

protected:
  double v_max_;
  double a_max_;
  double v_yaw_max_;
  double a_yaw_max_;
  double sampling_dt_;
};

} // end namespace mrp
