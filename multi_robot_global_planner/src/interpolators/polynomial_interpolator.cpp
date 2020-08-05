/**
 * @author Luca Bartolomei, V4RL
 * @date   01.08.2019
 */

#include "multi_robot_global_planner/interpolators/polynomial_interpolator.h"

namespace mrp {

PolynomialInterpolator::PolynomialInterpolator(const DynamicParams &params)
    : v_max_(params.v_max),
      a_max_(params.a_max),
      v_yaw_max_(params.v_yaw_max),
      a_yaw_max_(params.a_yaw_max),
      sampling_dt_(params.sampling_dt) {}

bool PolynomialInterpolator::interpolate(const GlobalPath &path,
                                         GlobalPath *interpolated_path) {
  // Clear output container
  interpolated_path->clear();

  // Step 1: generate yaws
  std::vector<double> yaws;
  for (size_t i = 0; i < path.size(); ++i) {
    yaws.push_back(path[i](3));
  }

  // Step 2: generate the verticies for the interpolation
  mav_trajectory_generation::Vertex::Vector vertices, vertices_yaw;
  const int dimension = 3;
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  const int derivative_to_optimize_yaw =
      mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;

  mav_trajectory_generation::Vertex vertex(dimension), vertex_yaw(1);
  vertex.makeStartOrEnd(path[0].head(3), derivative_to_optimize);

  double yaw_old = yaws[0];
  vertex_yaw.makeStartOrEnd(yaws[0], derivative_to_optimize_yaw);

  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  for (size_t i = 1; i < path.size() - 1; ++i) {
    mav_trajectory_generation::Vertex vertex_m(dimension), vertex_yaw_m(1);
    vertex_m.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION, path[i].head(3));

    if (std::abs(yaws[i] + 2 * M_PI - yaw_old) < std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] + 2 * M_PI;
    } else if (std::abs(yaws[i] - 2 * M_PI - yaw_old) <
               std::abs(yaws[i] - yaw_old)) {
      yaw_old = yaws[i] - 2 * M_PI;
    } else {
      yaw_old = yaws[i];
    }

    vertex_yaw_m.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION, yaw_old);
    vertices.push_back(vertex_m);
    vertices_yaw.push_back(vertex_yaw_m);
  }

  vertex.makeStartOrEnd(path.back().head(3), derivative_to_optimize);
  if (std::abs(yaws.back() + 2 * M_PI - yaw_old) <
      std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() + 2 * M_PI;
  } else if (std::abs(yaws.back() - 2 * M_PI - yaw_old) <
             std::abs(yaws.back() - yaw_old)) {
    yaw_old = yaws.back() - 2 * M_PI;
  } else {
    yaw_old = yaws.back();
  }
  vertex_yaw.makeStartOrEnd(yaw_old, derivative_to_optimize_yaw);
  vertices.push_back(vertex);
  vertices_yaw.push_back(vertex_yaw);

  // Step 3: obtain interpolated trajectory
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, v_max_, a_max_);
  std::vector<double> segment_times_yaw{
      mav_trajectory_generation::estimateSegmentTimes(vertices_yaw, v_yaw_max_,
                                                      a_yaw_max_)};

  for (int i = 0; i < segment_times.size(); ++i) {
    if (segment_times_yaw[i] > segment_times[i])
      segment_times[i] = segment_times_yaw[i];

    if (segment_times[i] < sampling_dt_) segment_times[i] = sampling_dt_;
  }

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  mav_trajectory_generation::PolynomialOptimization<N> opt_yaw(1);
  opt_yaw.setupFromVertices(vertices_yaw, segment_times,
                            derivative_to_optimize_yaw);
  opt_yaw.solveLinear();

  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory trajectory_yaw;
  mav_trajectory_generation::Trajectory trajectory_with_yaw;
  opt.getTrajectory(&trajectory);
  opt_yaw.getTrajectory(&trajectory_yaw);
  trajectory.getTrajectoryWithAppendedDimension(trajectory_yaw,
                                                &trajectory_with_yaw);

  mav_msgs::EigenTrajectoryPoint::Vector states;
  mav_trajectory_generation::sampleWholeTrajectory(trajectory_with_yaw,
                                                   sampling_dt_, &states);

  // Create output
  // Create a yaw which is feasible (TODO) - for the moment straight line
  createYawsFromStates(states, yaws);

  for (size_t i = 0; i < states.size(); ++i) {
    Eigen::Vector4d point(states[i].position_W(0), states[i].position_W(1),
                          states[i].position_W(2), states[i].getYaw());
    interpolated_path->push_back(point);
  }

  return true;
}

void PolynomialInterpolator::updateParameters(const double v_max,
                                              const double a_max,
                                              const double v_yaw_max,
                                              const double a_yaw_max,
                                              const double sampling_dt) {
  v_max_ = v_max;
  a_max_ = a_max;
  v_yaw_max_ = v_yaw_max;
  a_yaw_max_ = a_yaw_max;
  sampling_dt_ = sampling_dt;
}

void PolynomialInterpolator::createYawsFromStates(
    const mav_msgs::EigenTrajectoryPoint::Vector &states,
    std::vector<double> &yaws) {
  // Assure that the output vector is empty
  yaws.clear();
  yaws.resize(states.size(), 0.0);

  // Start filling the vector
  for (size_t i = 0; i < states.size() - 1; ++i) {
    Eigen::Vector2d dir =
        (states[i + 1].position_W.head<2>() - states[i].position_W.head<2>())
            .normalized();
    yaws[i] = std::atan2(dir(1), dir(0));
  }
  yaws.back() = yaws[yaws.size() - 2];
}

}  // end namespace mrp
