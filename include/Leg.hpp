#pragma once
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <numbers>

class Leg {
public:
  Leg(double lengths[3], double r_foot);

  // Forward Kinematics
  void fk(Eigen::Vector3d theta, Eigen::Vector3d &p);

  // Inverse Kinematics
  void ik(Eigen::Vector3d p, Eigen::Vector3d &theta);

  // Forward Dynamics
  void fd(Eigen::Vector3d theta, Eigen::Vector3d theta_d, Eigen::Vector3d &p_d);

  void trot(const Eigen::Vector2d &v, Eigen::Vector3d &p, useconds_t &dt);

  enum state_t { IDLE, TRAVEL, LIFT } state;

private:
  double l1, l2, l3, footRadius;
  double v_r = 0.2;          // m/s, return speed of foot during lift phase
  double travel = 0.05;      // m
  double lift_height = 0.05; // m
  Eigen::Vector2d dp = Eigen::Vector2d::Zero();
};