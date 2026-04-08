#include <minjerk_generator.h>

MinJerkGenerator::MinJerkGenerator(){
  t_mat_.row(0) << 0., 0., 0., 0., 0., 1.;
  t_mat_.row(2) << 0., 0., 0., 0., 1., 0.;
  t_mat_.row(4) << 0., 0., 0., 2., 0., 0.;
}

vola::trajectory_t MinJerkGenerator::get_trajectory(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& target_pos, double duration, double step_size){
  set_t_matrix_coeffs(duration);
  auto coeffs = get_traj_coefficients(curr_pos, target_pos);

  // turn into trajectory and return
  size_t sz = static_cast<size_t>(duration / step_size) + 1;
  vola::trajectory_t traj;
  traj.states.reserve(sz);

  double t = 0.;
  for (size_t step = 0; step < sz; ++step) {
    auto& state = traj.states.emplace_back();

    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    for (int axis = 0; axis < kNumAxes; ++axis) {
      const auto& c = coeffs.col(axis);

      const double a5 = c(0);
      const double a4 = c(1);
      const double a3 = c(2);
      const double a2 = c(3);
      const double a1 = c(4);
      const double a0 = c(5);

      state.pos[axis] =
          a5*t5 + a4*t4 + a3*t3 + a2*t2 + a1*t + a0;

      state.vel[axis] =
          5*a5*t4 + 4*a4*t3 + 3*a3*t2 + 2*a2*t + a1;

      state.acc[axis] =
          20*a5*t3 + 12*a4*t2 + 6*a3*t + 2*a2;

      state.jerk[axis] =
          60*a5*t2 + 24*a4*t + 6*a3;
    }

    state.time = t;
    t += step_size;
  }

  return traj;
}

void MinJerkGenerator::set_t_matrix_coeffs(double duration){
  double T1 = duration;
  double T2 = T1 * T1;
  double T3 = T2 * T1;
  double T4 = T3 * T1;
  double T5 = T4 * T1;

  t_mat_.row(1) << T5, T4, T3, T2, T1, 1.0;
  t_mat_.row(3) << 5*T4, 4*T3, 3*T2, 2*T1, 1, 0.;
  t_mat_.row(5) << 20*T3, 12*T2, 6*T1, 2., 0., 0.;
}

Eigen::Matrix<double, MinJerkGenerator::kDegree, MinJerkGenerator::kNumAxes> MinJerkGenerator::get_traj_coefficients(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& target_pos){
  Eigen::VectorXd bounds(6);
  Eigen::MatrixXd t_mat_inv = t_mat_.inverse();

  Eigen::Matrix<double, MinJerkGenerator::kDegree, MinJerkGenerator::kNumAxes> coeffs;
  for (int i = 0; i < MinJerkGenerator::kNumAxes; ++i) {
    // pos, vel, acc boundaries
    bounds << curr_pos(i), target_pos(i), 0., 0., 0., 0.;
    coeffs.col(i) = t_mat_inv * bounds;
  }

  return coeffs;
}

