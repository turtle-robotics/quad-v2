/**
 * @file spatial.hpp
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
} // namespace Eigen

/**
 * @brief Create screw axis
 * @param [in] q 3-vector screw axis
 * @param [in] s 3-vector screw offset
 * @param [in] h Screw pitch
 * @return 6-vector screw axis
 */
inline Eigen::Vector6d screwAxis(const Eigen::Vector3d &q,
                                 const Eigen::Vector3d &s,
                                 const double h = 0.0) {
  return (Eigen::Vector6d() << s, q.cross(s) + h * s).finished();
}

/**
 * @brief Create screw axis list
 * @param [in] q 3-vector screw axis
 * @param [in] s 3-vector screw offset
 * @param [in] h Screw pitch
 * @return 6-vector screw axis
 */
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

/**
 * @brief Create screw axis from points
 *
 * Create a screw axis from point p1 to point p2
 *
 * @param [in] p1 3-vector first point
 * @param [in] p2 3-vector second point
 * @return 6-vector screw axis
 */
inline Eigen::Vector6d pointsToScrew(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2) {
  return screwAxis(p1, (p2 - p1).normalized());
}

/**
 * @brief Create spatial inertia matrix
 *
 * @param [in] I 3x3 inertia matrix
 * @param [in] m mass
 * @return 6x6 spatial inertia matrix
 */
inline Eigen::Matrix6d makeG(const Eigen::Matrix3d &I, const double &m) {
  Eigen::Matrix6d G;
  G.topLeftCorner<3, 3>() = I;
  G.bottomRightCorner<3, 3>().diagonal().setConstant(m);
  return G;
}

/**
 * @brief Apply cubic time scaling
 *
 * @param [in] t Elapsed time (0 - dt)
 * @param [in] dt Total time duration
 * @return position along cubic spline
 */
inline double cubicTimeScaling(const double t, const double dt) {
  const double timeratio = t / dt;
  return 3 * std::pow(timeratio, 2) - 2 * std::pow(timeratio, 3);
}

/**
 * @brief Apply quintic time scaling
 *
 * @param [in] t Elapsed time (0 - dt)
 * @param [in] dt Total time duration
 * @return position along quintic spline
 */
inline double quinticTimeScaling(const double t, const double dt) {
  const double timeratio = t / dt;
  return 10 * std::pow(timeratio, 3) - 15 * std::pow(timeratio, 4) +
         6 * std::pow(timeratio, 5);
}