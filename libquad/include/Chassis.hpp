#pragma once

#include <Eigen/Geometry>
#include "Leg.hpp"

/**
 * Class Chassis
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 */

class Chassis {
public:
  Chassis(Eigen::Matrix<double, 6, 6> &G, Eigen::Isometry3d &M_home,
          std::array<Eigen::Isometry3d, 4> &M_chassis_shoulder)
      : G{G}, M_home{M_home}, M_chassis_shoulder{M_chassis_shoulder} {}

  void ik(const Eigen::Isometry3d &T_home_chassis,
          const std::array<Eigen::Isometry3d, 4> &T_leg,
          std::array<Eigen::Isometry3d, 4> &T_shoulder_leg);
  void force_dist();

private:
  const Eigen::Matrix<double, 6, 6> G; // kg*m^2

  // Body Frame in Home Configuration
  const Eigen::Isometry3d M_home;
  const std::array<Eigen::Isometry3d, 4> M_chassis_shoulder;
};