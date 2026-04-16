#pragma once

#include "Leg.hpp"
#include "spatial.hpp"
// #include <ModernRobotics>
#include <memory>

/**
 * Class Chassis
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 *
 * b is the chassis "body" frame, at the volumetric center of the chassis
 * 1 is the shoulder frame
 * 4 is the foot frame, different for each foot
 */

class Chassis {
public:
  Chassis(Eigen::Matrix6d &G, Eigen::Isometry3d &M0,
          std::array<Eigen::Isometry3d, 4> &M01)
      : G{G}, Ms{M0}, M01{M01} {}

  // Inverse Kinematics
  void ik();

  // Inverse Dynamics
  void id();

  // Margin of Static Stability
  void nvp();

  /* Chassis state */

  // Body Frame
  Eigen::Isometry3d Ts0;                  // SE(3)
  Eigen::Vector6d Vb;                     // rad/s, m/s
  Eigen::Vector6d dVb;                    // rad/s^2, m/s^2
  std::array<Eigen::Isometry3d, 4> T_leg; // SE(3)

  double K;

  // Leg Frame
  std::array<std::shared_ptr<Leg>, 4> legs;
  std::array<Eigen::Isometry3d, 4> T4;

private:
  const Eigen::Matrix6d G; // kg*m^2

  // Body Frame in Home Configuration
  const Eigen::Isometry3d Ms;                 // SE(3)
  const std::array<Eigen::Isometry3d, 4> M01; // SE(3)

  Eigen::Matrix<double, 12, 6> Jinv; //

  const Eigen::Vector3d g{0, 0, 9.81}; // m/s^2
};