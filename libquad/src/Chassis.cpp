#include "Chassis.hpp"
#include <vector>

// TODO: at some point, update T and V with V and dV and dt

bool Chassis::ik(const Eigen::Isometry3d &Ts0,
                 const std::array<Eigen::Isometry3d, 4> &T4,
                 std::array<Eigen::Vector3d, 4> &pf,
                 Eigen::Matrix<double, 6, 12> *Jinv) {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    Eigen::Isometry3d T04 = (Ms * Ts0).inverse() * T4[nleg];
    pf[nleg] = (M01[nleg].inverse() * T04).translation();
    if (Jinv == nullptr)
      continue;
    (*Jinv).block<3, 3>(0, nleg * 3) =
        T04.translation().asSkewSymmetric() * T04.linear();
    (*Jinv).block<3, 3>(3, nleg * 3) = T04.linear();
  }
  return true;
}

bool Chassis::ivk(const Eigen::Vector6d &Vb,
                  const Eigen::Matrix<double, 6, 12> &Jinv,
                  std::array<Eigen::Vector3d, 4> &vf) {
  Eigen::Vector<double, 12> vfStacked = Jinv.transpose() * Vb;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    vf[nleg] = vfStacked.segment<3>(nleg * 3);
  }
  return true;
}

bool Chassis::id(std::array<Eigen::Vector3d, 4> &ff) {
  Eigen::Vector<double, 12> ffStacked;
  ffStacked = Jinv.transpose() * Fb;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    ff[nleg] = ffStacked.segment<3>(nleg * 3);
  }
  return true;
}

bool Chassis::nvp(const Eigen::Vector6d &F,
                  const std::array<std::shared_ptr<Leg>, 4> &legs, double &K) {
  std::vector<unsigned> runningLegId;
  Eigen::Matrix<double, 3, 4> pfoot;
  for (unsigned nleg : {0, 1, 3, 2}) {
    pfoot.col(nleg) = legs[nleg]->pf;
    if (legs[nleg]->state == Leg::state_t::RUNNING)
      runningLegId.push_back(nleg);
  }

  if (runningLegId.size() < 2)
    return false;

  else if (runningLegId.size() == 3) {
    footPlane = footPlane.Through(pfoot.col(runningLegId[0]),
                                  pfoot.col(runningLegId[1]),
                                  pfoot.col(runningLegId[2]));
  }

  // Project feet onto plane
  else if (runningLegId.size() > 3) {
    // centroid of legs
    Eigen::Vector3d e = pfoot.rowwise().sum() / pfoot.cols();

    // recenter foot positions with centroid: p_i-p_centrid, compute U matrix
    // of SVD, select smallest col as normal vector
    Eigen::Vector3d n = ((-pfoot).colwise() + e)
                            .jacobiSvd<Eigen::ComputeFullU>()
                            .matrixU()
                            .rightCols<1>();
    footPlane.normal() = n;
    footPlane.offset() = -n.dot(e);

    // project foot positions onto plane
    // for (unsigned i = 0; i < pfoot.cols(); i++)
    //   pfoot.col(i) = footPlane.projection(pfoot.col(i));
  }
  for (unsigned i = 0; i < 4; i++)
    pfoot.col(i) = footPlane.projection(pfoot.col(i));

  // Create list of rollover axis screws
  Eigen::Matrix<double, 6, Eigen::Dynamic> Slist;
  unsigned ip = 3;
  for (unsigned i = 0; i < 4; ip = i, i++) {
    Slist.col(i) = pointsToScrew(legs[i]->pf, legs[ip]->pf);
  }

  // Create rollover normalized wrench
  Eigen::Vector6d Fhat;
  Fhat << F.tail<3>(), F.head<3>();
  Fhat /= F.head<3>().norm();

  // Margin of Static Stability K = S^T * Delta * F
  Eigen::VectorXd Klist = Slist.transpose() * Fhat;

  // Find minimum K
  K = Klist.minCoeff();
  ip = 3;
  double KfootMin = 0.0;
  for (int i = 0; i < Klist.rows(); ip = i, i++) {
    double Kfoot = Klist(ip) * Klist(i);
    if (Kfoot < KfootMin) {
      KfootMin = Kfoot;
      iRolloverLeg = i;
    }
  }
  return true;
}

void Chassis::run() {
  std::array<Eigen::Vector3d, 4> pf, vf, ff;
  ik(Ts0, T4, pf, &Jinv);
  ivk(Vb, Jinv, vf);
  id(ff);
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    legs[nleg]->pf = pf[nleg];
    legs[nleg]->vf = vf[nleg];
    legs[nleg]->ff = ff[nleg];
  }
  Eigen::Vector6d F = (Ms * Ts0).inverse().Ad().transpose() * Fb;
  nvp(F, legs, K);
  if (K < 0.1) { // TODO: Tune
    // legs[iRolloverLeg]->lift();
  }
}