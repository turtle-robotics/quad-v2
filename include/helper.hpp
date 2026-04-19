#pragma once

#include <Eigen/Dense>
#include <array>
#include <moteus.h>

#define LEG_ARRAY(type) std::array<type, 4>
#define LEG_JOINT_ARRAY(type) std::array<std::array<type, 3>, 4>

using PosCmd = mjbots::moteus::PositionMode::Command;
inline std::array<PosCmd, 3>
makePosCmd(Eigen::Vector3d theta, Eigen::Vector3d dtheta = {0.0, 0.0, 0.0},
           Eigen::Vector3d tau = {0.0, 0.0, 0.0}) {
  std::array<PosCmd, 3> posCmds;
  for (unsigned i = 0; i < 3; i++) {
    posCmds[i] = {.position = theta(i),
                  .velocity = dtheta(i),
                  .feedforward_torque = tau(i)};
  }
  return posCmds;
}