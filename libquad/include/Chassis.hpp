#pragma once

// #include <Eigen/Core>
#include <Eigen/Geometry>

const std::array<Eigen::Vector3d, 4> leg_dir{{{+1.0, -1.0, +1.0},
                                              {+1.0, +1.0, +1.0},
                                              {-1.0, -1.0, +1.0},
                                              {-1.0, +1.0, +1.0}}};

/**
 * Class Chassis
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 */

class Chassis {
public:
  Chassis(Eigen::Matrix<double, 6, 6> &G, Eigen::Isometry3d &M,
          std::array<Eigen::Isometry3d, 4> &T_chassis_shoulder)
      : G{G}, M{M}, T_chassis_shoulder{T_chassis_shoulder} {}

  void ik(const Eigen::Isometry3d &T_chassis,
          const std::array<Eigen::Isometry3d, 4> &leg_poses);
  void force_dist();

  Eigen::Isometry3d T_home_chassis;

private:
  const Eigen::Matrix<double, 6, 6> G; // kg*m^2

  // Body Frame in Home Configuration
  const Eigen::Isometry3d M;
  const std::array<Eigen::Isometry3d, 4> T_chassis_shoulder;
};