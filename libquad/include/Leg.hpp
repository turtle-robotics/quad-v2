#pragma once

#include "spatial.hpp"
#include <cmath>
#include <iostream>
#include <numbers>

/**
 * @brief Quadruped leg state
 *
 * The leg class contains gemotry and mass properties for one leg, the leg
 * state, and functions to operate on the leg state.
 */

constexpr int njoints = 3;

class Leg {
public:
  Leg(Eigen::Vector<double, njoints + 1> &l,
      Eigen::Matrix<double, 6, njoints> &Slist, Eigen::Isometry3d &M,
      std::array<Eigen::Isometry3d, njoints + 1> &Mlist,
      std::array<Eigen::Matrix6d, njoints> &Glist,
      Eigen::Matrix<double, njoints, 2> &thetaRange,
      Eigen::Vector<double, njoints> &dthetaMax,
      Eigen::Vector<double, njoints> &ddthetaMax,
      Eigen::Vector<double, njoints> &tauMax)
      : l{l}, Slist{Slist}, M{M}, Mlist{Mlist}, Glist{Glist},
        thetaRange{thetaRange}, dthetaMax{dthetaMax}, ddthetaMax{ddthetaMax},
        tauMax{tauMax} {};

  /**
   * @brief Forward Kinematics
   *
   * Uses thetalist to compute pf
   */
  void fk();

  // Inverse Kinematics
  bool ik();

  // Inverse Dynamics
  void id();

  // Initiate lift and set target
  void liftTo(const Eigen::Isometry3d &T);

  void run();

  /* Leg state */
  enum state_t {
    IDLE,
    HOMING,
    RUNNING,
    LIFT,
    PLACE
  } state = IDLE,
    statep = IDLE;

  // Foot space
  Eigen::Vector3d pf;     // m
  Eigen::Vector3d vf;     // m/s
  Eigen::Vector3d dvf;    // m/s^2
  Eigen::Vector3d ffoot;  // N
  Eigen::Vector3d g;      // m/s^2
  Eigen::Vector3d liftpf; // m
  double llift = 0.02;    // m TODO: Set from config

  // Joint space (updated by motor controller)
  Eigen::Vector<double, njoints> thetalist;   // rad
  Eigen::Vector<double, njoints> dthetalist;  // rad/s
  Eigen::Vector<double, njoints> ddthetalist; // rad/s^2
  Eigen::Vector<double, njoints> taulist;     // N*m

private:
  /* Leg geometry & mass properties */
  const Eigen::Vector<double, njoints + 1> l;             // m
  const Eigen::Matrix<double, 6, njoints> Slist;          //
  const Eigen::Isometry3d M;                              // SE(3)
  const std::array<Eigen::Isometry3d, njoints + 1> Mlist; // SE(3)
  const std::array<Eigen::Matrix6d, njoints> Glist;       // kg, kg*m^2

  /* Joint limits */
  const Eigen::Matrix<double, njoints, 2> thetaRange; // rad
  const Eigen::Vector<double, njoints> dthetaMax;     // rad/s
  const Eigen::Vector<double, njoints> ddthetaMax;    // rad/s^2
  const Eigen::Vector<double, njoints> tauMax;        // N*m

  /* Joint directions */
  const Eigen::Vector<double, njoints> thetadir{Slist(0, 0) * Slist(1, 1),
                                                Slist(1, 1), Slist(1, 1)};

  /* Inverse velocity kinematics */
  Eigen::Matrix<double, njoints, 3> Jinv; // rad/m

  /* Inverse dynamics */
  Eigen::Isometry3d Mi;                          // SE(3)
  Eigen::Matrix<double, 6, njoints> Ai;          //
  Eigen::Matrix<double, 6, njoints + 1> Vi;      // rad/s, m/s
  Eigen::Matrix<double, 6, njoints + 1> dVi;     // rad/s^2, m/s^2
  std::array<Eigen::Matrix6d, njoints + 1> AdTi; //
  Eigen::Vector6d Fi;                            // N*m, N
};