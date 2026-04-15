#pragma once

#include "Leg.hpp"
#include <Eigen/Geometry>
#include <ModernRobotics>

/**
 * Class Chassis
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 *
 * c0 is the chassis "body" frame, at the volumetric center of the chassis
 * l1 is the shoulder frame
 * l4 is the foot frame, different for each foot
 */

class Chassis {
public:
  Chassis(Eigen::Matrix<double, 6, 6> &G, Eigen::Isometry3d &M0,
          std::array<Eigen::Isometry3d, 4> &M01)
      : G{G}, M0{M0}, M01{M01} {}

  void ik(const Eigen::Isometry3d &T_B,
          const std::array<Eigen::Isometry3d, 4> &T_leg,
          std::array<Eigen::Isometry3d, 4> &T14, bool computeJacobian = true);
  void id();

  Eigen::Vector3d g{0, 0, 9.81}; // m/s^2

private:
  const Eigen::Matrix<double, 6, 6> G; // kg*m^2

  // Body Frame in Home Configuration
  const Eigen::Isometry3d M0;
  const std::array<Eigen::Isometry3d, 4> M01;

  Eigen::Matrix<double, 12, 6> Jinv;

  // void computeJinv(const Eigen::Translation3d &pf,
  //                  Eigen::Matrix<double, njoints, 3> &Jinv);
};