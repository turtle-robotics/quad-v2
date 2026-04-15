#include "Chassis.hpp"

void Chassis::ik(const Eigen::Isometry3d &T0,
                 const std::array<Eigen::Isometry3d, 4> &T4,
                 std::array<Eigen::Isometry3d, 4> &T14, bool computeJacobian) {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    T14[nleg] = (T4[nleg].inverse() * M0 * T0 * M01[nleg]).inverse();
    Eigen::Isometry3d T04 = M0 * T0 * M01[nleg];
    Jinv.block<3, 3>(nleg * 3, 0) =
        mr::VecToso3(T04.translation()) * T04.linear();
    Jinv.block<3, 3>(nleg * 3, 3) = T04.linear();
  }
}

void Chassis::id() {}