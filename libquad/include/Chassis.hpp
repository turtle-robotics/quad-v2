/**
 * @file Chassis.hpp
 */

#pragma once

#include "Leg.hpp"
#include "spatial.hpp"
#include <array>

/**
 * @brief Quadruped Chassis
 *
 * The chassis class contains gemotry and mass properties for the chassis,
 * the chassis state, and functions to operate on the chassis state.
 *
 * - {b} is the chassis "body" frame, at the volumetric center of the chassis
 * - {1} is the shoulder frame
 * - {4} is the foot frame, different for each foot
 */
class Chassis {
public:
  /**
   * @brief Chassis Constructor
   *
   * @param[in] G 6x6 spatial inertia matrix for the chassis
   * @param[in] Ms Home transform of the chassis in the base frame
   * @param[in] M01 Array of transforms from the chassis frame to the leg base
   * frames
   */
  Chassis(Eigen::Matrix6d &G, Eigen::Isometry3d &Ms,
          std::array<Eigen::Isometry3d, 4> &M01)
      : G{G}, Ms{Ms}, M01{M01} {}

  /**
   * @brief Inverse Kinematics
   *
   * Compute foot poses (and Jacobian inverse) at a given chassis pose
   *
   * @param[in] Ts0 SE(3) transform of the chassis in the chassis home frame
   * @param[in] T4 Array of SE(3) transforms of the legs in the base frame
   * @param[out] pf Array of 3-vector resultant foot positions in the leg base
   * frame
   * @param[out] Jinv 6x12 resultant Jacobian inverse (dfoot/dT)
   */
  bool ik(const Eigen::Isometry3d &Ts0,
          const std::array<Eigen::Isometry3d, 4> &T4,
          std::array<Eigen::Vector3d, 4> &pf,
          Eigen::Matrix<double, 6, 12> *Jinv);
  /**
   * @brief Inverse Velocity Kinematics
   *
   * Compute foot velocities for a given chassis twist
   *
   * @param[in] Vb 6-vector twist of the chassis body
   * @param[in] Jinv 6x12 Jacobian inverse (dfoot/dT)
   * @param[out] vf Array of 3-vector resultant foot velocities
   */
  bool ivk(const Eigen::Vector6d &Vb, const Eigen::Matrix<double, 6, 12> &Jinv,
           std::array<Eigen::Vector3d, 4> &vf);

  /**
   * @brief Inverse Dynamics
   *
   * Compute foot force from chassis state
   *
   * @param[out] ff Array of 3-vector resultant foot forces
   */
  bool id(std::array<Eigen::Vector3d, 4> &ff);

  /**
   * @brief Margin of Static Stability
   *
   * Compute the minimum Normalized Virtual Power required for rollover in the
   * current state
   *
   * @param[in] F Chassis wrench in the space frame
   * @param[in] pf Array of 3-vector resultant foot positions in the leg base
   * frame
   * @param[out] K Minimum Normalized Virtual Power
   */
  bool nvp(const Eigen::Vector6d &F,
           const std::array<std::shared_ptr<Leg>, 4> &legs, double &K);

  void run();

  /* Chassis state */
  Eigen::Isometry3d Ts0; ///< Tf chassis in home frame [SE(3)]
  Eigen::Vector6d Vb;    ///< Chassis twist [rad/s; m/s]
  Eigen::Vector6d dVb;   ///< Chassis spatial acceleration [rad/s^2; m/s^2]
  Eigen::Vector6d Fb;    ///< Chassis wrench [N*m; m]

  /* Rollover */
  double K;         ///< Lowest normalized virtual power
  int iRolloverLeg; ///< Index of leg with lowest bordering K (next leg to step)

  /* Legs */
  std::array<Eigen::Isometry3d, 4> T4; ///< Tf legs in the space frame [SE(3)]
  std::array<std::shared_ptr<Leg>, 4> legs; ///< Array of Leg shared pointers

  Eigen::Hyperplane<double, 3> footPlane; ///< Foot Plane

private:
  const Eigen::Matrix6d G; ///< Spatial inertia matrix [kg*m^2; m]

  // Body Frame in Home Configuration
  const Eigen::Isometry3d Ms; ///< Tf chassis home in the space frame [SE(3)]
  const std::array<Eigen::Isometry3d, 4>
      M01; ///< Tf legs in the chassis body frame [SE(3)]

  Eigen::Matrix<double, 6, 12> Jinv; ///< 6x12 Inverse Jacobian

  const Eigen::Vector3d g{0, 0, 9.81}; ///< Gravity vector [m/s^2]
};