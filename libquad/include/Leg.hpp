/**
 * @file Leg.hpp
 */

#pragma once

#include "spatial.hpp"
#include <cmath>
#include <iostream>
#include <numbers>

constexpr int njoints = 3;

/**
 * @brief Quadruped Leg
 *
 * The leg class contains gemotry and mass properties for one leg, the leg
 * state, and functions to operate on the leg state.
 */
class Leg {
public:
  Leg(Eigen::Vector<double, njoints + 1> &l,
      Eigen::Matrix<double, 6, njoints> &Slist, Eigen::Isometry3d &M,
      std::array<Eigen::Isometry3d, njoints + 1> &Mlist,
      std::array<Eigen::Matrix6d, njoints> &Glist,
      Eigen::Matrix<double, njoints, 2> &thetaRange,
      Eigen::Vector<double, njoints> &thetadMax,
      Eigen::Vector<double, njoints> &thetaddMax,
      Eigen::Vector<double, njoints> &tauMax)
      : l{l}, Slist{Slist}, M{M}, Mlist{Mlist}, Glist{Glist},
        thetaRange{thetaRange}, thetadMax{thetadMax}, thetaddMax{thetaddMax},
        tauMax{tauMax} {};

  /**
   * @brief Forward Kinematics
   *
   * Compute foot position from joint angles
   *
   * @param[in] thetalist 3-vector of joint angles
   * @param[out] pf 3-vector resultant foot position
   */
  bool fk(const Eigen::Vector3d &thetalist, Eigen::Vector3d &pf);

  /**
   * @brief Inverse Kinematics
   *
   * Compute joint angles (and Jacobian inverse) at a given foot position
   *
   * @param[in] pf 3-vector foot position
   * @param[out] thetalist 3-vector resultant joint angles
   * @param[out] Jinv 3x3 resultant Jacobian inverse (thetad/dp)
   */
  bool ik(const Eigen::Vector3d &pf, Eigen::Vector3d &thetalist,
          Eigen::Matrix3d *Jinv = nullptr);

  /**
   * @brief Inverse Velocity Kinematics
   *
   * Compute joint velocities for a given foot position velocity
   *
   * @param[in] vf 3-vector foot velocity
   * @param[in] Jinv 3x3 Jacobian inverse at the current position (thetad/dp)
   * @param[out] thetadlist 3-vector resultant joint angles
   */
  bool ivk(const Eigen::Vector3d &vf, const Eigen::Matrix3d &Jinv,
           Eigen::Vector3d &thetadlist);

  /**
   * @brief Inverse Dynamics
   *
   * Compute joint torque from foot state
   *
   * @param[in] T SE(3) foot pose
   */
  bool id(Eigen::Vector<double, njoints> &taulist);

  // Initiate lift and set target
  void liftTo(const Eigen::Isometry3d &T);

  /**
   * @brief Joint Trajectory
   *
   * Compute joint angles at a specified time on a joint trajectory
   *
   * @param[in] Tstart SE(3) beginning foot pose
   * @param[in] Tgoal SE(3) ending foot pose
   * @param[in] t0 Time at beginning of trajectory
   * @param[in] t Current time
   * @param[out] T SE(3) current foot pose
   */
  bool jointTrajectory(const Eigen::Isometry3d &Tstart,
                       const Eigen::Isometry3d &Tgoal, const double t0,
                       const double t, Eigen::Isometry3d &Tcurrent);

  /**
   * @brief Run foot operations
   */
  void run();

  /**
   * @brief Leg State
   */
  enum state_t {
    IDLE,
    HOMING,
    RUNNING,
    LIFT,
    PLACE
  } state = IDLE,
    statep = IDLE;

  // Foot space
  Eigen::Vector3d pf;    ///< Foot position [m]
  Eigen::Vector3d vf;    ///< Foot velocity [m/s]
  Eigen::Vector3d dvf;   ///< Foot acceleration [m/s^2]
  Eigen::Vector3d ffoot; ///< Foot force [N]
  Eigen::Vector3d g;     ///< Gravity [m/s^2]
  // Eigen::Vector3d liftpf; //< m
  // double llift = 0.02;    // m TODO: Set from config

  // Joint space (updated by motor controller)
  Eigen::Vector<double, njoints> thetalist;   // rad
  Eigen::Vector<double, njoints> thetadlist;  // rad/s
  Eigen::Vector<double, njoints> thetaddlist; // rad/s^2
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
  const Eigen::Vector<double, njoints> thetadMax;     // rad/s
  const Eigen::Vector<double, njoints> thetaddMax;    // rad/s^2
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