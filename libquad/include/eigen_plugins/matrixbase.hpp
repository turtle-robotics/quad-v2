/**
 * @file matrixbase.hpp
 *
 * Provides Eigen plugins for MatrixBase
 */

/**
 * @brief so(3) matrix exponent
 * @return SO(3) rotation matrix
 */
inline Matrix<Scalar, 3, 3> exp3() const {
  assert(RowsAtCompileTime == 3 && ColsAtCompileTime == 3);
  AngleAxis<Scalar> theta{&this};
  if (abs(theta.angle()) < 1e-6) {
    return Matrix<Scalar, 3, 3>::Identity();
  } else {
    Matrix<Scalar, 3, 3> omgmat{theta.axis().asSkewSymmetric()};
    return Matrix<Scalar, 3, 3>::Identity() + std::sin(theta.angle()) * omgmat +
           (1 - std::cos(theta.angle())) * omgmat * omgmat;
  }
}

/**
 * @brief se(3) matrix exponent
 * @return SE(3) transformation
 */
inline Transform<Scalar, 3, Isometry> exp6() const {
  assert(RowsAtCompileTime == 4 && ColsAtCompileTime == 4);
  // Extract the angular velocity vector from the transformation matrix
  AngleAxis<Scalar> theta{this->template block<3, 3>(0, 0)};
  Transform<Scalar, 3, Isometry> T;

  // If negligible rotation
  if (abs(theta.angle()) < 1e-6) {
    T.linear() = Matrix<Scalar, 3, 3>::Identity();
    T.translation() = this->template block<3, 1>(0, 3);
    return T;
  }

  Matrix<Scalar, 3, 3> omgmat{theta.axis().asSkewSymmetric()};
  T.linear() = Matrix<Scalar, 3, 3>::Identity() +
               std::sin(theta.angle()) * omgmat +
               (1 - std::cos(theta.angle())) * omgmat * omgmat;
  T.translation() =
      ((Matrix<Scalar, 3, 3>::Identity() * theta +
        (1 - std::cos(theta.angle())) * omgmat +
        (theta.angle() - std::sin(theta.angle())) * omgmat * omgmat) *
       this->template block<3, 1>(0, 3)) /
      theta.angle();
  return T;
}

/**
 * @brief 6-vector adjoint
 * @return 6x6 adjoint matrix
 */
inline Matrix<Scalar, 6, 6> ad() const {
  assert(RowsAtCompileTime == 6 && ColsAtCompileTime == 1);
  return (Matrix<Scalar, 6, 6>()
              << this->template segment<3>(0).asSkewSymmetric().toDenseMatrix(),
          Matrix<Scalar, 3, 3>::Zero(),
          this->template segment<3>(3).asSkewSymmetric().toDenseMatrix(),
          this->template segment<3>(0).asSkewSymmetric().toDenseMatrix())
      .finished();
}

/**
 * @brief 3-vector as se(3) matrix
 * @return se(3) matrix
 */
inline Matrix<Scalar, 4, 4> tose3() const {
  (
      Matrix<Scalar, 4, 4>()
          << this->template head<3>().asSkewSymmetric().toDenseMatrix(),
      this->template tail<3>(), 0, 0, 0, 0)
      .finished();
}