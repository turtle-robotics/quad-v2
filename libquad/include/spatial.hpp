#pragma once
#include <Eigen/Dense>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
} // namespace Eigen

inline Eigen::Vector6d screwAxis(const Eigen::Vector3d &q,
                                 const Eigen::Vector3d &s, const double &h) {
  return (Eigen::Vector6d() << s, q.cross(s) + h * s).finished();
}