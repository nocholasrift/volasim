#ifndef VOLASIM_NODE_MINJERK_GENERATOR_H
#define VOLASIM_NODE_MINJERK_GENERATOR_H

#include <types.h>
#include <Eigen/Core>

class MinJerkGenerator{
public:
  MinJerkGenerator();
  ~MinJerkGenerator() = default;

  vola::trajectory_t get_trajectory(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& target_pos, double time, double step_size=1e-2);

private:
  static constexpr uint8_t kDegree = 6;
  static constexpr uint8_t kNumAxes = 3;

  Eigen::Matrix<double, kDegree, kDegree> t_mat_;

private:
  void set_t_matrix_coeffs(double duration);

  Eigen::Matrix<double, kDegree, kNumAxes> get_traj_coefficients(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& target_pos);

};

#endif
