#pragma once
#include <Eigen/Geometry>
#include <chrono>
#include <iomanip>
#include <numbers>

#include "Leg.hpp"

class Chassis {
 public:
  Chassis(float chassisWidth, float chassisLength, float upperLegLength,
          float lowerLegLength, float shoulderWidth, float footRadius,

          Servo LFS, Servo LFU, Servo LFL,

          Servo RFS, Servo RFU, Servo RFL,

          Servo LBS, Servo LBU, Servo LBL,

          Servo RBS, Servo RBU, Servo RBL);
  void walk();
  Eigen::Vector2f velocity;
  float angularVelocity;

 private:
  float chassisWidth, chassisLength;
  float upperLegLength, lowerLegLength, shoulderWidth, footRadius;
  float reach;
  Eigen::Vector3f basePos;
  Leg LF, RF, LB, RB;
  std::chrono::high_resolution_clock::time_point start{
      std::chrono::high_resolution_clock::now()};

  Eigen::Vector3f p(Eigen::Vector2f v, float t, float n);
};