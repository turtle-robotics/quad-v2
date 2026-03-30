#pragma once

// #include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * Class Chassis
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 */

struct ChassisProperties {
  Eigen::Matrix<double, 6, 6> G; // kg*m^2, m
  Eigen::Translation3d home;     // m
};

class Chassis {
public:
  Chassis(ChassisProperties props) { Ts = props.home; }

  void ik(const Eigen::Isometry3d &chassis_frame,
          const std::array<Eigen::Translation3d, 4> &leg_poses);
  void force_dist();

private:
  Eigen::Isometry3d Ts;
  Eigen::Isometry3d Tsb;

  Eigen::Vector3d l;         // m
  double m;                  // kg
  Eigen::Vector3d I;         // kg*m^2
  Eigen::Translation3d home; // m
};