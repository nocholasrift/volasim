#include "matrix_utils.h"

namespace vola {
namespace utils {

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d mat;
  mat << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return mat;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& m) {
  Eigen::Vector3d vec;
  vec << m(2, 1), m(0, 2), m(1, 0);
  return vec;
}

}  // namespace utils
}  // namespace vola
