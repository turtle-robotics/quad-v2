#pragma once
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <numbers>

class Leg {
public:
  Leg(double l0, double l1, double l2, double r_foot);

  // Forward Kinematics
  void fk(Eigen::Vector3d theta, Eigen::Vector3d &p);

  // Inverse Kinematics
  void ik(Eigen::Vector3d p, Eigen::Vector3d &theta);

  // Forward Dynamics
  void fd(Eigen::Vector3d theta, Eigen::Vector3d theta_d, Eigen::Vector3d &p_d);

  void walk(Eigen::Vector2d v, useconds_t dt);

  enum state_t { IDLE, TRAVEL, LIFT, PLACE } state = IDLE;

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d theta = Eigen::Vector3d::Zero();

private:
  double l1, l2, l3, footRadius;
  double v_r = 0.2;     // m/s, return speed of foot during lift phase
  double v_min = 0.02;  // m/s, minimum speed of foot during lift phase
  double travel = 0.05; // m
  double travel2 = travel * travel; // m^2, precompute for efficiency
  double lift_height = 0.05;        // m
  Eigen::Vector2d dp = Eigen::Vector2d::Zero();
  const Eigen::Vector3d home_pos = Eigen::Vector3d(0.0, l1, 0.2);
  const Eigen::Vector3d lift_pos{home_pos.x(), home_pos.y(),
                                 home_pos.z() + lift_height};
};