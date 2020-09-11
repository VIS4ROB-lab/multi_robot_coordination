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

/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for polynomial interpolator
 * @date   05.08.2019
 */

#pragma once

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace mrp {

typedef std::vector<Eigen::VectorXd> LocalPath;

class PolynomialInterpolator {
 public:
  PolynomialInterpolator(const double v_max, const double a_max,
                         const double v_yaw_max, const double a_yaw_max,
                         const double sampling_dt);
  virtual ~PolynomialInterpolator() {}

  bool interpolate(
      const LocalPath &path, LocalPath *interpolated_path,
      const Eigen::Vector3d &initial_velocity = Eigen::Vector3d::Zero());

  void updateParameters(const double v_max, const double a_max,
                        const double v_yaw_max, const double a_yaw_max,
                        const double sampling_dt);

 private:
  void createYawsFromStates(
      const mav_msgs::EigenTrajectoryPoint::Vector &states,
      std::vector<double> &yaws);

 protected:
  double v_max_;
  double a_max_;
  double v_yaw_max_;
  double a_yaw_max_;
  double sampling_dt_;
};

}  // end namespace mrp
