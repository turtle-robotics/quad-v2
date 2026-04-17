#pragma once
#include <Eigen/Dense>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
} // namespace Eigen

inline Eigen::Vector6d screwAxis(const Eigen::Vector3d &q,
                                 const Eigen::Vector3d &s,
                                 const double &h = 0.0) {
  return (Eigen::Vector6d() << s, q.cross(s) + h * s).finished();
}

inline Eigen::Vector6d pointsToScrew(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2) {
  return screwAxis(p1, (p2 - p1).normalized());
}