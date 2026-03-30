#include "Chassis.hpp"
#include "Leg.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <fstream>
#include <string>
#include <ModernRobotics>
#include <yaml-cpp/yaml.h>

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

// JointProperties
template <> struct convert<JointProperties> {
  static bool decode(const Node &node, JointProperties &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    double m = node["mass"].as<double>(0);
    Eigen::Matrix3d I;
    I.diagonal() = node["inertia"].as<Eigen::Vector3d>(Eigen::Vector3d::Zero());
    rhs.G.topLeftCorner<3, 3>() = I;
    rhs.G.bottomRightCorner<3, 3>().diagonal().setConstant(m);
    rhs.thetaMin = node["thetaMin"].as<double>(-INFINITY);
    rhs.thetaMax = node["thetaMax"].as<double>(INFINITY);
    rhs.dthetaMax = node["dthetaMax"].as<double>(INFINITY);
    rhs.ddthetaMax = node["ddthetaMax"].as<double>(INFINITY);
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
    std::array<JointProperties, 4> joints{{shoulder, upper, lower, foot}};

    Eigen::Vector<double, 4> l;
    std::array<Eigen::Isometry3d, 4> Mlist;
    Eigen::Isometry3d M;
    Eigen::Matrix3d q, s;
    Eigen::Matrix<double, 6, 3> Slist;
    std::array<Eigen::Matrix<double, 6, 6>, 3> Glist;
    Eigen::Matrix<double, 3, 2> thetaRange;
    Eigen::Vector3d dthetaMax;
    Eigen::Vector3d ddthetaMax;
    Eigen::Vector3d tauMax;

    for (unsigned legNum = 0; legNum < 4; legNum++) {
      {
        unsigned j = 0;
        for (const auto &joint : joints) {
          Glist[j] = joint.G;
          l(j) = joint.l;
          thetaRange(j, 0) = joint.thetaMax;
          thetaRange(j, 1) = joint.thetaMin;
          dthetaMax(j) = joint.dthetaMax;
          ddthetaMax(j) = joint.ddthetaMax;
          tauMax(j) = joint.tauMax;
          j++;
        }
      }

      q << 0.0, 0.0, 0.0,                   // q1
          0.0, ySign[legNum] * l[0], 0.0,   // q2
          -l[1], ySign[legNum] * l[0], 0.0; // q3
      s << xSign[legNum], 0.0, 0.0,         // s1
          0.0, ySign[legNum], 0.0,          // s2
          0.0, ySign[legNum], 0.0;          // s3
      for (unsigned i = 0; i < 3; i++) {
        Slist.col(i) = mr::ScrewToAxis(q.row(i), s.row(i), 0.0);
      }

      M = Eigen::Translation3d{l[2] - l[1], ySign[legNum] * l[0], 0.0};

      Mlist[0] = Eigen::Translation3d{0.0, 0.0, 0.0};
      Mlist[1] = Eigen::Translation3d{0.0, ySign[legNum] * l[0], 0.0};
      Mlist[2] = Eigen::Translation3d{-l[1], ySign[legNum] * l[0], 0.0};
      Mlist[3] = Eigen::Translation3d{l[2] - l[1], ySign[legNum] * l[0], 0.0};
      rhs[legNum] = std::make_shared<Leg>(l, Slist, M, Mlist, Glist, thetaRange,
                                          dthetaMax, ddthetaMax, tauMax);
    }
    return true;
  }
};

// ChassisProperties
template <> struct convert<ChassisProperties> {
  static bool decode(const Node &node, ChassisProperties &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    double m = node["mass"].as<double>(0);
    Eigen::Matrix3d I;
    I.diagonal() = node["inertia"].as<Eigen::Vector3d>(Eigen::Vector3d::Zero());

    rhs.G.topLeftCorner<3, 3>() = I;
    rhs.G.bottomRightCorner<3, 3>().diagonal().setConstant(m);

    rhs.home.vector() =
        node["home"].as<Eigen::Vector3d>(Eigen::Vector3d::Identity());
    return true;
  }
};

// Chassis
template <> struct convert<Chassis> {
  static bool decode(const Node &node, Chassis &rhs) {
    if (!node.IsMap()) {
      return false;
    }

    rhs = Chassis(node.as<ChassisProperties>());
    return true;
  }
};
} // namespace YAML