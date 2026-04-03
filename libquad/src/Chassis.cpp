#include "Chassis.hpp"

void Chassis::ik(const Eigen::Isometry3d &T_chassis,
                 const std::array<Eigen::Isometry3d, 4> &leg_poses) {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    leg_poses[nleg] = (t_leg.inverse() * M *T_chassis* T_chassis_shoulder).inverse();
  }
}