#pragma once
#include <Eigen/Geometry>
#include <chrono>
#include <iomanip>
#include <numbers>

#include "Leg.hpp"

class Chassis
{
public:
  Chassis(float chassisWidth, float chassisLength,
          Leg LF, Leg RF, Leg LB, Leg RB);
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