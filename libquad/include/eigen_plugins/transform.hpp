/**
 * @file transform.hpp
 *
 * Provides Eigen plugins for Transform
 */

/**
 * @brief SE(3) Adjoint matrix
 * @return 6x6 Adjoint matrix
 */
inline Matrix<Scalar, 6, 6> Ad() const {
  return (Matrix<Scalar, 6, 6>() << linear(), Matrix<Scalar, 3, 3>::Zero(),
          linear() * translation().asSkewSymmetric(), linear())
      .finished();
}