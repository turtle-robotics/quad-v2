#pragma once
#include <Eigen/Core>
#include <iostream>
#include <numbers>
#include <cmath>

class Leg
{
public:
  Leg(float lengths[3], float r_foot);

  // Forward Kinematics
  void fk(Eigen::Vector3f theta, Eigen::Vector3f &p);

  // Inverse Kinematics
  void ik(Eigen::Vector3f p, Eigen::Vector3f &theta);

private:
  float l1, l2, l3, footRadius;
};