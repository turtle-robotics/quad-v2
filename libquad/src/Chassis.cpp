#include "Chassis.hpp"

void Chassis::ik() {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    Eigen::Isometry3d T04 = (Ms * Ts0).inverse() * T4[nleg];
    legs[nleg]->pf = (M01[nleg].inverse() * T04).translation();

    Jinv.block<3, 3>(nleg * 3, 0) = T04.translation().asSkewSymmetric()*T04.linear();
    Jinv.block<3, 3>(nleg * 3, 3) = T04.linear();
  }
  Eigen::Vector<double, 12> vf = Jinv * Vb;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    legs[nleg]->vf = vf.segment<3>(nleg * 3);
  }
}

void Chassis::id() {}

void Chassis::nvp() {
  unsigned nlegp = 0;
  for (unsigned nleg : {1, 3, 2, 0}) {
    if (legs[nleg]->state != Leg::state_t::RUNNING)
      continue;

    legs[nleg]->pf - legs[nlegp]->pf;
  }

  for (auto leg : legs) {
    if (leg->state == Leg::state_t::RUNNING) {
    }
  }

  // for (unsigned nleg = 0; nleg < 4; nleg++) {
  //   Ms *Ts0.inverse()
  // }
}