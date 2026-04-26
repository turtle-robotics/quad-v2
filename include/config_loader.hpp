#pragma once

#include "Chassis.hpp"
#include "Leg.hpp"
#include "Robot.hpp"
#include "spatial.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#if defined(__aarch64__)
#include <pi3hat_moteus_transport.h>
#endif

// Internal convention: LF: 0, RF: 1, LB: 2, RB: 3
constexpr double xSign[4]{1.0, 1.0, -1.0, -1.0};
constexpr double ySign[4]{-1.0, 1.0, -1.0, 1.0};

const std::array<Eigen::Vector3d, 4> leg_dir{{{+1.0, -1.0, -1.0},
                                              {+1.0, +1.0, +1.0},
                                              {-1.0, -1.0, -1.0},
                                              {-1.0, +1.0, +1.0}}};

struct JointProperties {
  double l;                  // m
  Eigen::Matrix6d G;         // kg*m^2, kg
  double thetaMax, thetaMin; // rad
  double thetadMax;          // rad/s
  double thetaddMax;         // rad/s^2
  double tauMax;             // N*m
};

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

template <> struct convert<std::shared_ptr<Robot::JointPose>> {
  static bool decode(const Node &node, std::shared_ptr<Robot::JointPose> &rhs) {
    if (!node.IsSequence() || !node.size() == 4) {
      return false;
    }
    for (int nleg = 0; nleg < 4; nleg++) {
      for (int njoint = 0; njoint < 3; njoint++) {
        (*rhs)[nleg][njoint] = node[nleg][njoint].as<double>();
      }
    }
    return true;
  }
};

// Robot
template <> struct convert<std::shared_ptr<Robot>> {
  static bool decode(const Node &node, std::shared_ptr<Robot> &rhs) {
    if (!node.IsMap() || !node["joints"].IsMap() || !node["servomap"].IsMap()) {
      return false;
    }

#if defined(__aarch64__)
    const auto transport =
        node["servomap"].as<std::shared_ptr<pi3hat::Pi3HatMoteusTransport>>();
#endif

    Robot::Motors motors;

    // Create motor objects
    for (int nleg = 0; nleg < 4; nleg++) {
      for (int njoint = 0; njoint < 3; njoint++) {
        int can_id = (nleg + 1) * 10 + njoint + 1;
        motors[nleg][njoint] = std::make_shared<moteus::Controller>([&]() {
          moteus::Controller::Options coptions;
#if defined(__aarch64__)
          coptions.transport = transport,
#endif
          coptions.id = can_id;
          return coptions;
        }());
      }
    }

    rhs = std::make_shared<Robot>(
        node["chassis"].as<std::shared_ptr<Chassis>>(),
        node["joints"].as<std::array<std::shared_ptr<Leg>, 4>>(),
        node["gamepad"].as<std::shared_ptr<Teleop>>(), motors);
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

// Leg
template <> struct convert<std::array<std::shared_ptr<Leg>, 4>> {
  static bool decode(const Node &node,
                     std::array<std::shared_ptr<Leg>, 4> &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    JointProperties shoulder = node["shoulder"].as<JointProperties>();
    JointProperties upper = node["upper"].as<JointProperties>();
    JointProperties lower = node["lower"].as<JointProperties>();
    JointProperties foot = node["foot"].as<JointProperties>();
    std::array<JointProperties, 3> joints{{shoulder, upper, lower}};

    Eigen::Vector4d l;
    std::array<Eigen::Isometry3d, 4> Mlist;
    Eigen::Isometry3d M;
    Eigen::Matrix<double, 6, 3> Slist;
    std::array<Eigen::Matrix6d, 3> Glist;
    Eigen::Matrix<double, 3, 2> thetaRange;
    Eigen::Vector3d thetadMax;
    Eigen::Vector3d thetaddMax;
    Eigen::Vector3d tauMax;

    for (unsigned nLeg = 0; nLeg < 4; nLeg++) {
      {
        unsigned j = 0;
        for (const auto &joint : joints) {
          Glist[j] = joint.G;
          l(j) = joint.l;
          thetaRange(j, 0) = joint.thetaMax;
          thetaRange(j, 1) = joint.thetaMin;
          thetadMax(j) = joint.thetadMax;
          thetaddMax(j) = joint.thetaddMax;
          tauMax(j) = joint.tauMax;
          j++;
        }
        l(j) = foot.l;
      }
      Slist = makeSlist(l.head<3>(), xSign[nLeg], ySign[nLeg]);
      M.setIdentity();
      M.translation() = Eigen::Vector3d{l[2] - l[1], ySign[nLeg] * l[0], 0.0};

      Mlist[0].setIdentity();
      Mlist[1].setIdentity();
      Mlist[2].setIdentity();
      Mlist[3].setIdentity();

      Mlist[0].translation() = Eigen::Vector3d{0.0, 0.0, 0.0};
      Mlist[1].translation() = Eigen::Vector3d{0.0, ySign[nLeg] * l[0], 0.0};
      Mlist[2].translation() = Eigen::Vector3d{-l[1], ySign[nLeg] * l[0], 0.0};
      Mlist[3].translation() =
          Eigen::Vector3d{l[2] - l[1], ySign[nLeg] * l[0], 0.0};
      rhs[nLeg] = std::make_shared<Leg>(l, Slist, M, Mlist, Glist, thetaRange,
                                        thetadMax, thetaddMax, tauMax);
    }
    return true;
  }
};

// Chassis
template <> struct convert<std::shared_ptr<Chassis>> {
  static bool decode(const Node &node, std::shared_ptr<Chassis> &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    double m = node["mass"].as<double>(0.0);
    Eigen::Matrix3d I = node["inertia"]
                            .as<Eigen::Vector3d>(Eigen::Vector3d::Zero())
                            .asDiagonal();
    Eigen::Matrix6d G = makeG(I, m);
    Eigen::Vector3d leg_offset = node["leg_offset"].as<Eigen::Vector3d>();
    Eigen::Isometry3d M;
    std::array<Eigen::Isometry3d, 4> T_chassis_shoulder;

    M.translation() = node["home"].as<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    for (unsigned nleg = 0; nleg < 4; nleg++) {
      T_chassis_shoulder[nleg].setIdentity();
      T_chassis_shoulder[nleg].translation() =
          leg_dir[nleg].cwiseProduct(leg_offset);
    }

    rhs = std::make_shared<Chassis>(G, M, T_chassis_shoulder);
    return true;
  }
};

template <> struct convert<std::shared_ptr<Teleop>> {
  static bool decode(const Node &node, std::shared_ptr<Teleop> &rhs) {
    if (!node.IsScalar()) {
      return false;
    }
    rhs = std::make_shared<Teleop>(node.as<std::string>(""));
    return true;
  }
};

// Moteus pi3hat Transport
#if defined(__aarch64__)
template <> struct convert<std::shared_ptr<pi3hat::Pi3HatMoteusTransport>> {
  static bool decode(const Node &node,
                     std::shared_ptr<pi3hat::Pi3HatMoteusTransport> &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    pi3hat::Pi3HatMoteusTransport::Options toptions;
    for (const auto &conf_bus_id : node) {
      for (const auto &conf_servo_id : conf_bus_id.second) {
        toptions.servo_map[conf_servo_id.as<int>()] =
            conf_bus_id.first.as<int>();
      }
    }
    rhs = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);

    return true;
  }
};
#endif

} // namespace YAML