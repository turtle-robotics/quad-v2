/**
 * @file Leg.hpp
 * @brief Contains the quadruped Leg class
 */

#pragma once

#include "spatial.hpp"
#include <chrono>
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
  /**
   * @brief Leg Constructor
   *
   * @param[in] l 4-vector of leg lengths
   * @param[in] Slist 6x3 list of joint screw axes
   * @param[in] M Transform of the leg at zero joint angles
   * @param[in] Mlist Array of 4 transforms between joints at zero joint angles,
   * beginning from the base (Identity)
   * @param[in] Glist Array of 3 6x6 spatial inertia matrices for the leg links
   * @param[in] thetaRange 3x2 matrix of joint angle limits [low, high]
   * @param[in] thetadMax 3-vector of joint velocity limits
   * @param[in] thetaddMax 3-vector of joint acceleration limits
   * @param[in] tauMax 3-vector of joint torque limits
   */
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
        tauMax{tauMax} {

    T_deploy.translation() = Eigen::Vector3d{0, 0, 0};
  };

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
   * @param[out] Jinv 3x3 resultant Jacobian inverse (dtheta/dp)
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
   * @param[out] taulist joint torques
   */
  bool id(Eigen::Vector<double, njoints> &taulist);

  // Initiate lift and set target
  void liftTo(const Eigen::Isometry3d &T);

  /**
   * @brief Joint Trajectory
   *
   * Compute joint angles at a specified time on a joint trajectory
   *
   * @param[in] theta0 beginning joint angles
   * @param[in] thetaf ending joint angles
   * @param[in] t Current time, normalized to trajectory [0,1]
   * @param[out] theta current joint angles
   */
  bool jointTrajectory(const Eigen::Vector<double, njoints> &theta0,
                       const Eigen::Vector<double, njoints> &thetaf,
                       const double t, Eigen::Vector<double, njoints> &theta);

  /**
   * @brief Joint Trajectory
   *
   * Compute joint angles at a specified time on a joint trajectory
   *
   * @param[in] Tstart SE(3) beginning foot pose
   * @param[in] Tgoal SE(3) ending foot pose
   * @param[in] t Current time, normalized to trajectory [0,1]
   * @param[out] T SE(3) current foot pose
   */
  bool jointTrajectory(const Eigen::Isometry3d &Tstart,
                       const Eigen::Isometry3d &Tgoal, const double t,
                       Eigen::Isometry3d &Tcurrent);

  /**
   * @brief Run foot operations
   */
  void run();

  /**
   * @brief Leg State
   */
  enum State {
    IDLE,
    HOMING,
    DEPLOY,
    RUNNING,
    LIFT,
    PLACE
  } state = IDLE,
    statep = IDLE;

  /* Foot space */
  Eigen::Isometry3d Tf{Eigen::Isometry3d::Identity()}; ///< Foot pose SE(3)
  Eigen::Ref<Eigen::Vector3d> pf{Tf.translation()};    ///< Foot position [m]
  Eigen::Vector3d vf;                                  ///< Foot velocity [m/s]
  Eigen::Vector3d dvf; ///< Foot acceleration [m/s^2]
  Eigen::Vector3d ff;  ///< Foot force [N]
  Eigen::Vector3d g;   ///< Gravity [m/s^2]

  /* Joint space (updated by motor controller) */
  Eigen::Vector<double, njoints> thetalist;  ///< Joint angle [rad]
  Eigen::Vector<double, njoints> thetadlist; ///< Joint angular velocity [rad/s]
  Eigen::Vector<double, njoints>
      thetaddlist; ///< Joint angular acceleration [rad/s^2]
  Eigen::Vector<double, njoints> taulist; ///< Joint torque [N*m]

  Eigen::Isometry3d T_stow;   ///< Stow foot pose SE(3)
  Eigen::Isometry3d T_deploy; ///< Deploy foot pose SE(3)

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

  /* Trajectory Generation */
  std::chrono::high_resolution_clock::time_point traj_start;
};