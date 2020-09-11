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
 * @date   01.08.2019
 */

#include "multi_robot_global_planner/interpolators/ramp_interpolator.h"

namespace mrp {

RampInterpolator::RampInterpolator(const double v_max, const double a_max,
                                   const double sampling_dt)
    : v_max_(v_max), a_max_(a_max), sampling_dt_(sampling_dt) {}

bool RampInterpolator::interpolate(const GlobalPath &path,
                                   GlobalPath *interpolated_path) {
  // Initialize variables
  double time = 0.0;
  double velocity = 0.0;
  interpolated_path->clear();

  for (size_t i = 0; i < path.size() - 1; ++i) {
    // Extract start and end point for the current segment
    Eigen::Vector3d start = path[i].head(3);
    Eigen::Vector3d end = path[i + 1].head(3);

    // Extract the current yaw
    Eigen::Vector2d direction = (end.head(2) - start.head(2)).normalized();
    double yaw = std::atan2(direction(1), direction(0));

    // Figure out what the total segment time will be.
    double total_segment_distance = (end - start).norm();
    // Total time needed to get to max speed (or go from max speed to 0).
    double min_acceleration_time = v_max_ / a_max_;
    // The amount of distance covered during the acceleration (or decceleration
    // process).
    double min_acceleration_distance =
        v_max_ * min_acceleration_time -
        0.5 * a_max_ * min_acceleration_time * min_acceleration_time;

    double total_segment_time = 0.0;
    // Case 1: time is shorter than the acceleration and decceleration time.
    if (total_segment_distance < 2 * min_acceleration_distance) {
      total_segment_time = 2 * std::sqrt(total_segment_distance / a_max_);
    } else {
      // Case 2: time is longer than accel + deccel time.
      total_segment_time =
          2 * min_acceleration_time +
          (total_segment_distance - 2 * min_acceleration_distance) / v_max_;
    }
    size_t num_elements = total_segment_time / sampling_dt_;
    Eigen::Vector3d path_direction = (end - start).normalized();

    // Start filling the current segment (do not care about distance and grad)
    Eigen::Vector4d point;

    // Treat this as a 1D problem since it is. ;)
    double position = 0.0;

    // Separate the time between total time and local segment time
    double current_time_trajectory = time;
    int64_t current_time_segment = 0.0;

    for (size_t j = 0; j < num_elements; ++j) {
      // Integrate velocity to get position.
      position += velocity * sampling_dt_;

      // Figure out if we're accelerating, deccelerating, or neither.
      // Handle Case 1 first:
      if (total_segment_time < min_acceleration_time * 2) {
        if (current_time_segment < total_segment_time / 2.0) {
          velocity += a_max_ * sampling_dt_;
        } else {
          velocity -= a_max_ * sampling_dt_;
        }
      } else {
        // Case 2
        if (position <= min_acceleration_distance) {
          velocity += a_max_ * sampling_dt_;
        } else if ((total_segment_distance - position) <=
                   min_acceleration_distance) {
          velocity -= a_max_ * sampling_dt_;
        }
      }

      // Make sure to meet constraints (could be passed/missed due to
      // discretization error).
      if (position > total_segment_distance) {
        position = total_segment_distance;
      }
      if (velocity > v_max_) {
        velocity = v_max_;
      }
      if (velocity < 0) {
        velocity = 0;
      }

      // Save the interpolated path
      point.head(3) = start + path_direction * position;
      point(3) = yaw;
      interpolated_path->push_back(point);

      // Update times
      current_time_trajectory += sampling_dt_;
      current_time_segment += sampling_dt_;
    }

    time = current_time_trajectory + sampling_dt_;
  }

  // Add end point
  Eigen::Vector3d position_end = path.back().head(3);
  Eigen::Vector3d direction_end =
      (position_end - path[path.size() - 2].head(3)).normalized();
  double yaw_end = std::atan2(direction_end(1), direction_end(0));

  Eigen::Vector4d point;
  point.head(3) = position_end;
  point(3) = yaw_end;

  interpolated_path->push_back(point);
  return true;
}

void RampInterpolator::updateParameters(const double v_max, const double a_max,
                                        const double sampling_dt) {
  v_max_ = v_max;
  a_max_ = a_max;
  sampling_dt_ = sampling_dt;
}

}  // namespace mrp
