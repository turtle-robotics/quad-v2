#include "Chassis.hpp"

void Chassis::ik() {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    Eigen::Isometry3d T04 = (Ms * Ts0).inverse() * T4[nleg];
    legs[nleg]->pf = (M01[nleg].inverse() * T04).translation();

    Jinv.block<3, 3>(0, nleg * 3) =
        T04.translation().asSkewSymmetric() * T04.linear();
    Jinv.block<3, 3>(3, nleg * 3) = T04.linear();
  }
  Eigen::Vector<double, 12> vf = Jinv.transpose() * Vb;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    legs[nleg]->vf = vf.segment<3>(nleg * 3);
  }
}

void Chassis::id() {

  Eigen::Vector<double, 12> ff;
  ff = Jinv.transpose() * Fb;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    legs[nleg]->ffoot = ff.segment<3>(nleg * 3);
  }
}

void Chassis::nvp() {
  Eigen::Matrix<double, 3, Eigen::Dynamic> pfoot;
  for (unsigned nleg = 0; nleg < 4; nleg++)
    if (legs[nleg]->state == Leg::state_t::RUNNING)
      pfoot.col(nleg) = legs[nleg]->pf; // Add leg to list

  if (pfoot.cols() < 2)
    return;

  // Project feet onto plane
  if (pfoot.cols() > 3) {
    // centroid of legs
    Eigen::Vector3d e = pfoot.colwise().sum() / pfoot.cols();

    // recenter foot positions with centroid: p_i-p_centrid, compute U matrix of
    // SVD, select smallest col as normal vector
    Eigen::Vector3d n = ((-pfoot).colwise() + e)
                            .jacobiSvd<Eigen::ComputeFullU>()
                            .matrixU()
                            .rightCols<1>();
    footPlane.normal() = n;
    footPlane.offset() = -n.dot(e);

    // project foot positions onto plane
    for (unsigned i = 0; i < pfoot.cols(); i++)
      pfoot.col(i) = footPlane.projection(pfoot.col(i));
  }

  Eigen::Matrix<double, 6, Eigen::Dynamic> Slist;
  unsigned nlegp = 0;
  for (unsigned nleg : {1, 3, 2, 0}) {
    Slist.col(nleg) = pointsToScrew(legs[nleg]->pf, legs[nlegp]->pf);
    nlegp = nleg;
  }

  Eigen::Vector6d F;
  F << F.tail<3>(), Fb.head<3>();
  F /= Fb.head<3>().norm();
  // K = S^T * Delta * F
  K = (Slist.transpose() * F).minCoeff();
}