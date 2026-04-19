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

template <int n>
inline Eigen::Matrix<double, 6, n> screwAxis(
    const Eigen::Matrix<double, 3, n> &q, const Eigen::Matrix<double, 3, n> &s,
    const Eigen::Vector<double, n> &h = Eigen::Vector<double, n>::Zero()) {
  assert(q.cols() == s.cols());
  Eigen::Matrix<double, 6, n> Slist =
      Eigen::Matrix<double, 6, n>::Zero(6, q.cols());
  for (unsigned i = 0; i < q.cols(); i++)
    Slist.col(i) = screwAxis(q.row(i), s.row(i), h(i));
  return Slist;
}

inline Eigen::Vector6d pointsToScrew(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2) {
  return screwAxis(p1, (p2 - p1).normalized());
}

inline Eigen::Matrix6d makeG(const Eigen::Matrix3d &I, const double &m) {
  Eigen::Matrix6d G;
  G.topLeftCorner<3, 3>() = I;
  G.bottomRightCorner<3, 3>().diagonal().setConstant(m);
  return G;
}