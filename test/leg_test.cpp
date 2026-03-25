#include "Leg.hpp"
#include <Eigen/Core>
#include <cassert>
#include <gtest/gtest.h>
#include <map>

int main() {
  std::map<int, Leg *> legs;

  for (int i = 1; i <= 4; i++)
    legs[i] = new Leg(0.04715084, 0.192, 0.192, 0.020);

  const std::map<int, Eigen::Vector3d> foot_positions{
      {1, {0.000, 0.010, 0.200}}, //
      {2, {0.000, 0.010, 0.200}}, //
      {3, {0.000, 0.010, 0.200}}, //
      {4, {0.000, 0.010, 0.200}}, //
  };

  std::map<int, double> jointPose;
  for (auto &pose_pair : foot_positions) {
    int leg_id = pose_pair.first;
    Eigen::Vector3d theta = Eigen::Vector3d::Zero();
    legs[leg_id]->ik(pose_pair.second, theta);
    ::printf("theta: %f, %f, %f\n", theta[0], theta[1], theta[2]);
  }

  return 0;
}