/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for ramp (linear velocity) interpolator
 * @date   01.08.2019
 */

#pragma once

#include "multi_robot_global_planner/utils/typedefs.h"

namespace mrp {

class RampInterpolator {
 public:
  RampInterpolator(const double v_max, const double a_max,
                   const double sampling_dt);
  virtual ~RampInterpolator() {}

  bool interpolate(const GlobalPath &path, GlobalPath *interpolated_path);
  void updateParameters(const double v_max, const double a_max,
                        const double sampling_dt);

 protected:
  double v_max_;
  double a_max_;
  double sampling_dt_;
};

}  // end namespace mrp
