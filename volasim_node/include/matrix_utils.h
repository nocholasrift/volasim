#ifndef VOLASIM_NODE_MATRIX_UTILS_H
#define VOLASIM_NODE_MATRIX_UTILS_H

#include <Eigen/Core>

namespace vola {
namespace utils {

Eigen::Matrix3d hat(const Eigen::Vector3d& v);

Eigen::Vector3d vee(const Eigen::Matrix3d& m);

}  // namespace utils
}  // namespace vola

#endif
