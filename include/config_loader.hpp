/**
 * @file config_loader.hpp
 * @brief Creates robot objects with configuration file data
 */

#pragma once

#include "Chassis.hpp"
#include "HID.hpp"
#include "Leg.hpp"
#include "Robot.hpp"
#include "spatial.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <memory>
#include <pi3hat_moteus_transport.h>
#include <string>
#include <yaml-cpp/yaml.h>

/// Internal convention: LF: 0, RF: 1, LB: 2, RB: 3
constexpr double xSign[4]{1.0, 1.0, -1.0, -1.0};
constexpr double ySign[4]{-1.0, 1.0, -1.0, 1.0};

const LegArray<Eigen::Vector3d> leg_dir{{{+1.0, -1.0, -1.0},
                                         {+1.0, +1.0, +1.0},
                                         {-1.0, -1.0, -1.0},
                                         {-1.0, +1.0, +1.0}}};

/**
 * @brief Stores attributes of a single joint
 */
struct JointProperties {
  double l;          /// Length [m]
  Eigen::Matrix6d G; /// Spatial Inertia [kg*m^2, kg]
  double thetaMin;   /// Joint position minimum limit [rad]
  double thetaMax;   /// Joint position maximum limit [rad]
  double thetadMax;  /// Joint velocity limit [rad/s]
  double thetaddMax; /// Joint acceleration limit [rad/s^2]
  double tauMax;     /// Joint torque limit [N*m]
};

/**
 * @brief Create a 6xn list of screw axes
 * @param l 3-vector of leg lengths
 * @param xSign sign multiplier in the x direction
 * @param ySign sign multiplier in the y direction
 * @return Screw axis list
 */
inline Eigen::Matrix<double, 6, 3>
makeSlist(const Eigen::Vector3d &l, const double xSign, const double ySign) {
  Eigen::Matrix3d q, s;
  q << 0.0, 0.0, 0.0,           // q1
      0.0, ySign * l[0], 0.0,   // q2
      -l[1], ySign * l[0], 0.0; // q3
  s << xSign, 0.0, 0.0,         // s1
      0.0, ySign, 0.0,          // s2
      0.0, ySign, 0.0;          // s3
  return screwAxis(q, s);
}

namespace YAML {

template <class Scalar, int n> struct convert<Eigen::Vector<Scalar, n>> {
  static bool decode(const Node &node, Eigen::Vector<Scalar, n> &rhs) {
    if (!node.IsSequence() || !node.size() == n && n != -1) {
      return false;
    }
    rhs = Eigen::Vector<Scalar, n>::Zero(node.size());
    for (unsigned i = 0; i < node.size(); i++) {
      rhs(i) = node[i].as<Scalar>();
    }
    return true;
  }
};

template <> struct convert<JointPose> {
  static bool decode(const Node &node, JointPose &rhs) {
    if (!node.IsSequence() || !node.size() == 4) {
      return false;
    }
    for (int nleg = 0; nleg < 4; nleg++) {
      for (int njoint = 0; njoint < 3; njoint++) {
        rhs(nleg * 3 + njoint) = node[nleg][njoint].as<double>();
      }
    }
    return true;
  }
};

// JointProperties
template <> struct convert<JointProperties> {
  static bool decode(const Node &node, JointProperties &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    double m = node["mass"].as<double>(0);
    Eigen::Matrix3d I = node["inertia"]
                            .as<Eigen::Vector3d>(Eigen::Vector3d::Zero())
                            .asDiagonal();
    rhs.G = makeG(I, m);
    rhs.thetaMin = node["thetaMin"].as<double>(-INFINITY);
    rhs.thetaMax = node["thetaMax"].as<double>(INFINITY);
    rhs.thetadMax = node["thetadMax"].as<double>(INFINITY);
    rhs.thetaddMax = node["thetaddMax"].as<double>(INFINITY);
    rhs.tauMax = node["tauMax"].as<double>(INFINITY);
    return true;
  }
};
} // namespace YAML

/**
 * @brief Construct an mjbots Moteus Pi3 Hat Transport object from a YAML node
 * @param node YAML configuration
 * @return Pi3 hat transport object
 */
inline std::shared_ptr<pi3hat::Pi3HatMoteusTransport>
makePi3HatTransport(const YAML::Node &node) {
  if (!node.IsMap()) {
    throw "Pi3 Hat Transport YAML is incomplete";
  }

  pi3hat::Pi3HatMoteusTransport::Options toptions;

  // IMU options
  toptions.attitude_rate_hz = 100;
  toptions.mounting_deg.pitch = 0;
  toptions.mounting_deg.yaw = 0;
  toptions.mounting_deg.roll = 0;
  // Servo map
  for (const auto &conf_bus_id : node) {
    for (const auto &conf_servo_id : conf_bus_id.second) {
      toptions.servo_map[conf_servo_id.as<int>()] = conf_bus_id.first.as<int>();
    }
  }
  return std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
}

/**
 * @brief Construct an HID object from a YAML node
 * @param node YAML configuration
 * @return HID object
 */
inline HID makeHID(const YAML::Node &node) {
  if (!node.IsScalar()) {
    throw "HID YAML is incomplete";
  }
  return HID{node.as<std::string>("")};
}

/**
 * @brief Construct an array of Leg objects from a YAML node
 * @param node YAML configuration
 * @return Array of Leg objects
 */
inline LegArray<std::shared_ptr<Leg>> makeLegs(const YAML::Node &node) {
  if (!node.IsMap()) {
    throw "Leg YAML is incomplete";
  }
  LegArray<std::shared_ptr<Leg>> legs;

  JointProperties shoulder = node["shoulder"].as<JointProperties>();
  JointProperties upper = node["upper"].as<JointProperties>();
  JointProperties lower = node["lower"].as<JointProperties>();
  JointProperties foot = node["foot"].as<JointProperties>();
  JointArray<JointProperties> joints{{shoulder, upper, lower}};

  Eigen::Vector4d l;
  std::array<Eigen::Isometry3d, 4> Mlist;
  Eigen::Isometry3d M;
  Eigen::Matrix<double, 6, 3> Slist;
  JointArray<Eigen::Matrix6d> Glist;
  Eigen::Matrix<double, 3, 2> thetaRange;
  Eigen::Vector3d thetadMax;
  Eigen::Vector3d thetaddMax;
  Eigen::Vector3d tauMax;

  for (unsigned nleg = 0; nleg < 4; nleg++) {
    {
      unsigned j = 0;
      for (const auto &joint : joints) {
        Glist[j] = joint.G;
        l(j) = joint.l;
        thetaRange(j, 0) =
            leg_dir[nleg](j) * std::min(joint.thetaMax, joint.thetaMin);
        thetaRange(j, 1) =
            leg_dir[nleg](j) * std::max(joint.thetaMax, joint.thetaMin);
        thetadMax(j) = joint.thetadMax;
        thetaddMax(j) = joint.thetaddMax;
        tauMax(j) = joint.tauMax;
        j++;
      }
      l(j) = foot.l;
    }
    Slist = makeSlist(l.head<3>(), xSign[nleg], ySign[nleg]);
    M.setIdentity();
    M.translation() = Eigen::Vector3d{l[2] - l[1], ySign[nleg] * l[0], 0.0};

    Mlist[0].setIdentity();
    Mlist[1].setIdentity();
    Mlist[2].setIdentity();
    Mlist[3].setIdentity();

    Mlist[0].translation() = Eigen::Vector3d{0.0, 0.0, 0.0};
    Mlist[1].translation() = Eigen::Vector3d{0.0, ySign[nleg] * l[0], 0.0};
    Mlist[2].translation() = Eigen::Vector3d{-l[1], ySign[nleg] * l[0], 0.0};
    Mlist[3].translation() =
        Eigen::Vector3d{l[2] - l[1], ySign[nleg] * l[0], 0.0};
    legs[nleg] = std::make_shared<Leg>(l, Slist, M, Mlist, Glist, thetaRange,
                                       thetadMax, thetaddMax, tauMax);
  }
  return legs;
};

/**
 * @brief Construct a Chassis object from a YAML node
 * @param node YAML configuration
 * @return Chassis object
 */
inline Chassis makeChassis(const YAML::Node &node) {
  if (!node.IsMap()) {
    throw "Chassis YAML is incomplete";
  }

  double m = node["mass"].as<double>(0.0);
  Eigen::Matrix3d I =
      node["inertia"].as<Eigen::Vector3d>(Eigen::Vector3d::Zero()).asDiagonal();
  Eigen::Matrix6d G = makeG(I, m);
  Eigen::Vector3d leg_offset = node["leg_offset"].as<Eigen::Vector3d>();
  Eigen::Isometry3d M;
  LegArray<Eigen::Isometry3d> T_chassis_shoulder;

  M.translation() = node["home"].as<Eigen::Vector3d>(Eigen::Vector3d::Zero());

  for (unsigned nleg = 0; nleg < 4; nleg++) {
    T_chassis_shoulder[nleg].setIdentity();
    T_chassis_shoulder[nleg].translation() =
        leg_dir[nleg].cwiseProduct(leg_offset);
  }

  return Chassis{G, M, T_chassis_shoulder};
}

/**
 * @brief Construct a Robot object from a YAML node
 * @param node YAML configuration
 * @return Robot object
 */
inline std::shared_ptr<Robot> makeRobot(const YAML::Node &node) {
  if (!node.IsMap() || !node["joints"].IsMap() || !node["motors"].IsMap()) {
    throw "Robot YAML is incomplete";
  }

  const auto legs = makeLegs(node["joints"]);

  const auto transport = makePi3HatTransport(node["motors"]["servomap"]);

  std::string canIDMap[4]{"LF", "RF", "LB", "RB"};
  std::map<uint32_t, unsigned> can_map;

  // Create motor objects
  Motors motors;
  for (unsigned i = 0; i < 12; i++) {
    unsigned can_id =
        node["motors"]["map"][canIDMap[i / 3]][i % 3].as<unsigned>();
    motors[i] = std::make_shared<moteus::Controller>([&]() {
      moteus::Controller::Options coptions;
      coptions.transport = transport;
      coptions.id = can_id;
      return coptions;
    }());
    can_map.insert({can_id, i});
  }

  return std::make_shared<Robot>(makeChassis(node["chassis"]), legs,
                                 makeHID(node["gamepad"]), motors, can_map,
                                 transport);
};