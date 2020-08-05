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
