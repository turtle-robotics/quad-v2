#include "Leg.hpp"
#include <Eigen/Core>
#include <gtest/gtest.h>

Eigen::Vector<double, njoints + 1> l;
Eigen::Matrix<double, 6, njoints> Slist;
Eigen::Isometry3d M;
std::array<Eigen::Isometry3d, njoints + 1> Mlist;
std::array<Eigen::Matrix<double, 6, 6>, njoints> Glist;
Eigen::Matrix<double, njoints, 2> thetaRange;
Eigen::Vector<double, njoints> dthetaMax;
Eigen::Vector<double, njoints> ddthetaMax;
Eigen::Vector<double, njoints> tauMax;

Leg makeLeg() {
  l << 0.115, 0.2, 0.2, 0.020;
  Slist << 1, 0, 0, //
      0, 1, 1,      //
      0, 0, 0,      //
      0, 0, 0,      //
      0, 0, 0,      //
      0, 0, -l[2];
  M.translation() = Eigen::Vector3d{0, l[0], 0};
  Mlist[0] = Eigen::Isometry3d::Identity();
  Mlist[1].translation() = Eigen::Vector3d{0, l[0], 0};
  Mlist[2].translation() = Eigen::Vector3d{-l[1], 0, 0};
  Mlist[3].translation() = Eigen::Vector3d{l[2], 0, 0};
  Glist[0] = Eigen::Matrix<double, 6, 6>::Zero();
  Glist[1] = Eigen::Matrix<double, 6, 6>::Zero();
  Glist[2] = Eigen::Matrix<double, 6, 6>::Zero();

  // TODO: Include feasible limits
  thetaRange = Eigen::Matrix<double, njoints, 2>::Zero();
  dthetaMax = Eigen::Vector<double, njoints>::Zero();
  ddthetaMax = Eigen::Vector<double, njoints>::Zero();
  tauMax = Eigen::Vector<double, njoints>::Zero();
  return {l, Slist, M, Mlist, Glist, thetaRange, dthetaMax, ddthetaMax, tauMax};
}

TEST(QUADLegTest, IKTest) {
  Leg leg = makeLeg();

  Eigen::Translation3d pf{0.0, 0.0, 0.25};
  Eigen::Vector3d thetalist;
  const Eigen::Vector3d expected_thetalist{0.5236, 0.5213, -1.0425};

  leg.ik(pf, thetalist);

  EXPECT_TRUE(thetalist.isApprox(expected_thetalist, 1e-4))
      << "Values should match:\n"
      << thetalist << "\n\n"
      << expected_thetalist;
}

TEST(QUADLegTest, IDTest) {
  Leg leg = makeLeg();
  const Eigen::Vector3d expected_thetalist{0.5236, 0.5213, -1.0425};

  leg.ik(pf, thetalist);

  EXPECT_TRUE(thetalist.isApprox(expected_thetalist, 1e-4))
      << "Values should match:\n"
      << thetalist << "\n\n"
      << expected_thetalist;
}
