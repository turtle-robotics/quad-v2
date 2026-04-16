#pragma once
#include <Eigen/Dense>

// #define EIGEN_MATRIX_PLUGIN "matrix_plugin.hpp"
// #define EIGEN_TRANSFORM_PLUGIN "eigen_plugins/transform.hpp"

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
} // namespace Eigen
/*
inline void exp(const Eigen::Matrix3d &so3mat, Eigen::Matrix3d &R) {
  Eigen::AngleAxisd theta{so3mat};
  if (abs(theta.angle()) < 1e-6) {
    R = Eigen::Matrix3d::Identity();
    return;
  } else {
    Eigen::Matrix3d omgmat{theta.axis().asSkewSymmetric()};
    R = Eigen::Matrix3d::Identity() + std::sin(theta.angle()) * omgmat +
        (1 - std::cos(theta.angle())) * omgmat * omgmat;
    return;
  }
}

inline Eigen::Matrix3d exp(const Eigen::Matrix3d &so3mat) {
  Eigen::Matrix3d R;
  exp(so3mat, R);
  return R;
}

inline void exp(const Eigen::Matrix4d &se3mat, Eigen::Isometry3d &T) {
  // Extract the angular velocity vector from the transformation matrix
  Eigen::AngleAxisd theta{se3mat.topLeftCorner<3, 3>()};

  // If negligible rotation
  if (abs(theta.angle()) < 1e-6) {
    T.linear() = Eigen::Matrix3d::Identity();
    T.translation() = se3mat.block<3, 1>(0, 3);
    return;
  }

  Eigen::Matrix3d omgmat{theta.axis().asSkewSymmetric()};
  T.linear() = Eigen::Matrix3d::Identity() + std::sin(theta.angle()) * omgmat +
               (1 - std::cos(theta.angle())) * omgmat * omgmat;
  T.translation() =
      ((Eigen::Matrix3d::Identity() * theta +
        (1 - std::cos(theta.angle())) * omgmat +
        (theta.angle() - std::sin(theta.angle())) * omgmat * omgmat) *
       se3mat.block<3, 1>(0, 3)) /
      theta.angle();
}

inline Eigen::Isometry3d exp(const Eigen::Matrix4d &se3mat) {
  Eigen::Isometry3d T;
  exp(se3mat, T);
  return T;
}

inline void ad(const Eigen::Vector6d &V, Eigen::Matrix6d &adV) {
  adV << V.segment<3>(0).asSkewSymmetric().toDenseMatrix(),
      Eigen::Matrix3d::Zero(),
      V.segment<3>(3).asSkewSymmetric().toDenseMatrix(),
      V.segment<3>(0).asSkewSymmetric().toDenseMatrix();
}

inline Eigen::Matrix6d ad(const Eigen::Vector6d &V) {
  Eigen::Matrix6d adV;
  ad(V, adV);
  return adV;
}

// template <int n = 1>
// inline void ad(const Eigen::Vector<double, 6 * n> &V,
//                Eigen::Matrix<double, 6 * n, 6 * n> &adV) {
//   adV = Eigen::Matrix<double, 6 * n, 6 * n>::Zero(6 * n, 6 * n);
//   for (unsigned i = 0; i < n; i += 6) {
//     adV.block<3, 3>(i, i) = V.template segment<3>(i).asSkewSymmetric();
//     adV.block<3, 3>(i + 3, i) = V.template segment<3>(i +
//     3).asSkewSymmetric(); adV.block<3, 3>(i + 3, i + 3) = adV.block<3, 3>(i,
//     i);
//   }
// }

inline void Ad(const Eigen::Isometry3d &T, Eigen::Matrix6d &AdT) {
  AdT << T.linear(), Eigen::Matrix3d::Zero(),
      T.linear() * T.translation().asSkewSymmetric(), T.linear();
}

inline Eigen::Matrix<double, 6, 6> Ad(const Eigen::Isometry3d &T) {
  Eigen::Matrix<double, 6, 6> AdT;
  Ad(T, AdT);
  return AdT;
}

// template <int n = 1>
// inline void Ad(const std::array<Eigen::Isometry3d, n> &T,
//                Eigen::Matrix<double, 6 * n, 6 * n> &AdT) {
//   AdT = Eigen::Matrix<double, 6 * n, 6 * n>::Zero(6 * n, 6 * n);
//   for (unsigned i = 0; i < n; i += 6) {
//     Ad(T[i], AdT.block<6, 6>(i, i));
//   }
// }
*/

// inline void se3(const Eigen::Vector6d &V, Eigen::Matrix4d &se3mat) {
//   se3mat << V.head<3>().asSkewSymmetric().toDenseMatrix(), V.tail<3>(), 0, 0, 0,
//       0;
// }

// inline Eigen::Matrix4d se3(const Eigen::Vector6d &V) {
//   Eigen::Matrix4d se3mat;
//   se3(V, se3mat);
//   return se3mat;
// }

inline Eigen::Vector6d screwAxis(const Eigen::Vector3d &q,
                                 const Eigen::Vector3d &s, const double &h) {
  return (Eigen::Vector6d() << s, q.cross(s) + h * s).finished();
}