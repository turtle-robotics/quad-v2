#pragma once

#include <Eigen/Dense>
#include <array>
#include <moteus.h>

template <typename type> using LegArray = std::array<type, 4>;
template <typename type> using JointArray = std::array<type, njoints>;
template <typename type> using LegJointArray = std::array<type, 4 * njoints>;

typedef LegJointArray<std::shared_ptr<mjbots::moteus::Controller>> Motors;
typedef Eigen::Matrix<double, njoints, 4> JointPose;

using PosCmd = mjbots::moteus::PositionMode::Command;
inline JointArray<PosCmd> makePosCmd(Eigen::Vector3d theta,
                                     Eigen::Vector3d thetad = {0.0, 0.0, 0.0},
                                     Eigen::Vector3d tau = {0.0, 0.0, 0.0}) {
  JointArray<PosCmd> posCmds;
  for (unsigned i = 0; i < 3; i++) {
    posCmds[i] = {.position = theta(i),
                  .velocity = thetad(i),
                  .feedforward_torque = tau(i)};
  }
  return posCmds;
}