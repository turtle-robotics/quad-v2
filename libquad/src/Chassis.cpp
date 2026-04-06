#include "Chassis.hpp"

void Chassis::ik(const Eigen::Isometry3d &T_home_chassis,
                 const std::array<Eigen::Isometry3d, 4> &T_leg,
                 std::array<Eigen::Isometry3d, 4> &T_shoulder_leg) {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    T_shoulder_leg[nleg] = (T_leg[nleg].inverse() * M_home * T_home_chassis *
                            M_chassis_shoulder[nleg])
                               .inverse();
  }
}