#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ModernRobotics>
#include <cmath>
#include <iostream>
#include <numbers>

/**
 * Class Leg
 * The leg class contains gemotry and mass properties for one leg, the leg
 * state, and functions to operate on the leg state.
 */

constexpr int njoints = 3;

// Internal convention: LF: 0, RF: 1, LB: 2, RB: 3
constexpr double xSign[4]{1.0, 1.0, -1.0, -1.0};
constexpr double ySign[4]{-1.0, 1.0, -1.0, 1.0};

struct JointProperties {
  double l;                      // m
  Eigen::Matrix<double, 6, 6> G; // kg*m^2, kg
  double thetaMax, thetaMin;     // rad
  double dthetaMax;              // rad/s
  double ddthetaMax;             // rad/s^2
  double tauMax;                 // N*m
};

struct LegProperties {
  /* Leg geometry & mass properties */
  double l1, l2, l3, rf;                                  // m
  Eigen::Matrix<double, 6, njoints> Slist;                //
  Eigen::Isometry3d M;                                    // SE(3)
  std::array<Eigen::Isometry3d, njoints + 1> Mlist;       // SE(3)
  std::array<Eigen::Matrix<double, 6, 6>, njoints> Glist; // kg, kg*m^2

  /* Joint limits */
  Eigen::Matrix<double, njoints, 2> thetaRange; // (max, min) rad
  Eigen::Vector<double, njoints> dthetaMax;     // (max, min) rad/s
  Eigen::Vector<double, njoints> ddthetaMax;    // (max, min) rad/s^2
  Eigen::Vector<double, njoints> tauMax;        // (max, min) N*m
};

class Leg {
public:
  Leg(Eigen::Vector<double, njoints+1> &l,
      Eigen::Matrix<double, 6, njoints> &Slist,
      Eigen::Isometry3d &M,
      std::array<Eigen::Isometry3d, njoints + 1> &Mlist,
      std::array<Eigen::Matrix<double, 6, 6>, njoints> &Glist,
      Eigen::Matrix<double, njoints, 2> &thetaRange,
      Eigen::Vector<double, njoints> &dthetaMax,
      Eigen::Vector<double, njoints> &ddthetaMax,
      Eigen::Vector<double, njoints> &tauMax)
      : l{l}, Slist{Slist}, M{M}, Mlist{Mlist}, Glist{Glist},
        thetaRange{thetaRange}, dthetaMax{dthetaMax}, ddthetaMax{ddthetaMax},
        tauMax{tauMax} {};

  // Forward Kinematics
  void fk();

  // Inverse Kinematics
  void ik();

  // Forward Dynamics
  // void fd(const Eigen::Vector3d &theta, const Eigen::Vector3d &theta_d,
  //         Eigen::Vector3d &p_d);

  // Inverse Dynamics
  void id(const Eigen::Vector3d &g);

  void run();

  // void home();
  // void walk(const Eigen::Vector3d &v, useconds_t dt);

  /* Leg state */
  enum state_t { IDLE, HOMING, RUNNING, LIFT, PLACE } state = IDLE;

  // Foot space
  Eigen::Translation3d pf;      // m
  Eigen::Vector<double, 6> Vf;  // rad/s, m/s
  Eigen::Vector<double, 6> dVf; // rad/s^2, m/s^2
  Eigen::Vector3d ff;           // N

  // Joint space (updated by motor controller)
  Eigen::Vector<double, njoints> thetalist;   // rad
  Eigen::Vector<double, njoints> dthetalist;  // rad/s
  Eigen::Vector<double, njoints> ddthetalist; // rad/s^2
  Eigen::Vector<double, njoints> taulist;     // N*m

private:
  /* Leg geometry & mass properties */
  const Eigen::Vector<double, njoints+1> l;
  // const double &l1 = l(0), &l2 = l(1), &l3 = l(2), &rf = l(3);  // m
  const Eigen::Matrix<double, 6, njoints> Slist;                //
  const Eigen::Isometry3d M;                                    // SE(3)
  const std::array<Eigen::Isometry3d, njoints + 1> Mlist;       // SE(3)
  const std::array<Eigen::Matrix<double, 6, 6>, njoints> Glist; // kg, kg*m^2

  /* Joint limits */
  const Eigen::Matrix<double, njoints, 2> thetaRange; // rad
  const Eigen::Vector<double, njoints> dthetaMax;     // rad/s
  const Eigen::Vector<double, njoints> ddthetaMax;    // rad/s^2
  const Eigen::Vector<double, njoints> tauMax;        // N*m

  // double v_r = 0.2;     // m/s, return speed of foot during lift phase
  // double v_min = 0.02;  // m/s, minimum speed of foot during lift phase
  // double travel = 0.05; // m
  // double travel2 = travel * travel; // m^2, precompute for efficiency
  // double lift_height = 0.100;       // m
  // Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  // const Eigen::Vector3d home_pos{leg_num < 3 ? -0.05 : 0.05, l1, 0.30};
  // const Eigen::Vector3d lift_pos{home_pos.x(), home_pos.y(),
  //                                home_pos.z() - lift_height};

  // PosFmt fmt_home{
  //     .position = Resolution::kIgnore,
  //     .velocity = Resolution::kFloat,
  //     .maximum_torque = Resolution::kFloat,
  //     .ignore_position_bounds = Resolution::kFloat,
  // };
  // PosCmd cmd_home{
  //     .velocity = -0.1,
  //     .maximum_torque = 3.5,
  //     .ignore_position_bounds = 1.0,
  // };
  // PosFmt fmt_deploy{
  //     .position = Resolution::kFloat,
  //     .velocity = Resolution::kIgnore,
  //     .maximum_torque = Resolution::kFloat,
  //     .velocity_limit = Resolution::kFloat,
  // };
  // PosCmd cmd_deploy{
  //     .maximum_torque = 2.5,
  //     .velocity_limit = 0.5,
  // };
};