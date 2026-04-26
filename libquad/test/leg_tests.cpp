#include "Leg.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>

const std::array<Eigen::Vector3d, 4> leg_dir{{{+1.0, -1.0, -1.0},
                                              {+1.0, +1.0, +1.0},
                                              {-1.0, -1.0, -1.0},
                                              {-1.0, +1.0, +1.0}}};

Leg makeLeg(int i) {
  Eigen::Vector<double, njoints + 1> l;
  Eigen::Matrix<double, 6, njoints> Slist;
  Eigen::Isometry3d M;
  std::array<Eigen::Isometry3d, njoints + 1> Mlist;
  std::array<Eigen::Matrix6d, njoints> Glist;
  Eigen::Matrix<double, njoints, 2> thetaRange;
  Eigen::Vector<double, njoints> thetadMax;
  Eigen::Vector<double, njoints> thetaddMax;
  Eigen::Vector<double, njoints> tauMax;

  l << 0.115, 0.2, 0.2, 0.020;
  Slist << 1, 0, 0, //
      0, 1, 1,      //
      0, 0, 0,      //
      0, 0, 0,      //
      0, 0, 0,      //
      0, 0, -l[2];
  Slist *= leg_dir[i].asDiagonal();
  Mlist[0] = Eigen::Isometry3d::Identity();
  Mlist[1].translation() = Eigen::Vector3d{0, l[0] * leg_dir[i][1], 0};
  Mlist[2].translation() = Eigen::Vector3d{-l[1], 0, 0};
  Mlist[3].translation() = Eigen::Vector3d{l[2], 0, 0};
  M = Mlist[0] * Mlist[1] * Mlist[2] * Mlist[3];
  Glist[0] = Eigen::Matrix6d::Zero();
  Glist[1] = Eigen::Matrix6d::Zero();
  Glist[2] = Eigen::Matrix6d::Zero();

  // TODO: Include feasible limits
  thetaRange = Eigen::Matrix<double, njoints, 2>::Zero();
  thetadMax = Eigen::Vector<double, njoints>::Zero();
  thetaddMax = Eigen::Vector<double, njoints>::Zero();
  tauMax = Eigen::Vector<double, njoints>::Zero();
  return {l, Slist, M, Mlist, Glist, thetaRange, thetadMax, thetaddMax, tauMax};
}

TEST(QUADLegTest, IKTest) {
  Leg leg1 = makeLeg(0);
  Leg leg2 = makeLeg(1);
  Leg leg3 = makeLeg(2);
  Leg leg4 = makeLeg(3);

  Eigen::Vector3d pf{0.0, 0.0, 0.25};
  Eigen::Vector3d thetalist1, thetalist2, thetalist3, thetalist4;
  const Eigen::Vector3d expected_thetalist1{-0.5236, -0.5213, +1.0425};
  const Eigen::Vector3d expected_thetalist2{+0.5236, +0.5213, -1.0425};
  const Eigen::Vector3d expected_thetalist3{+0.5236, -0.5213, +1.0425};
  const Eigen::Vector3d expected_thetalist4{-0.5236, +0.5213, -1.0425};

  leg1.ik(pf, thetalist1);
  leg2.ik(pf, thetalist2);
  leg3.ik(pf, thetalist3);
  leg4.ik(pf, thetalist4);

  EXPECT_TRUE(thetalist1.isApprox(expected_thetalist1, 1e-4))
      << "Values should match:\n"
      << thetalist1 << "\n\n"
      << expected_thetalist1;

  EXPECT_TRUE(thetalist2.isApprox(expected_thetalist2, 1e-4))
      << "Values should match:\n"
      << thetalist2 << "\n\n"
      << expected_thetalist2;

  EXPECT_TRUE(thetalist3.isApprox(expected_thetalist3, 1e-4))
      << "Values should match:\n"
      << thetalist3 << "\n\n"
      << expected_thetalist3;

  EXPECT_TRUE(thetalist4.isApprox(expected_thetalist4, 1e-4))
      << "Values should match:\n"
      << thetalist4 << "\n\n"
      << expected_thetalist4;
}

// TEST(QUADLegTest, IVKTest) {
//   Leg leg1 = makeLeg(0);
//   Leg leg2 = makeLeg(1);
//   Leg leg3 = makeLeg(2);
//   Leg leg4 = makeLeg(3);
//   Eigen::Vector3d pf{0.0, 0.0, 0.25};
//   Eigen::Vector3d thetadlist1, thetadlist2, thetadlist3, thetadlist4;
//   Eigen::Vector3d vf{0.1, 0.2, 0.3};
//   leg1.ivk(vf, pf, thetadlist1);
//   leg2.ivk(vf, pf, thetadlist2);
//   leg3.ivk(vf, pf, thetadlist3);
//   leg4.ivk(vf, pf, thetadlist4);

//   Eigen::Vector3d expected_thetadlist1{+1.6226, -0.4966, +1.9973};
//   Eigen::Vector3d expected_thetadlist2{-1.6226, +0.4966, -1.9973};
//   Eigen::Vector3d expected_thetadlist3{+1.6226, -1.5007, +1.9973};
//   Eigen::Vector3d expected_thetadlist4{-1.6226, +1.5007, -1.9973};

//   EXPECT_TRUE(thetadlist1.isApprox(expected_thetadlist1, 1e-4))
//       << "Values should match:\n"
//       << thetadlist1 << "\n\n"
//       << expected_thetadlist1;

//   EXPECT_TRUE(thetadlist2.isApprox(expected_thetadlist2, 1e-4))
//       << "Values should match:\n"
//       << thetadlist2 << "\n\n"
//       << expected_thetadlist2;

//   EXPECT_TRUE(thetadlist3.isApprox(expected_thetadlist3, 1e-4))
//       << "Values should match:\n"
//       << thetadlist3 << "\n\n"
//       << expected_thetadlist3;

//   EXPECT_TRUE(thetadlist4.isApprox(expected_thetadlist4, 1e-4))
//       << "Values should match:\n"
//       << thetadlist4 << "\n\n"
//       << expected_thetadlist4;
// }