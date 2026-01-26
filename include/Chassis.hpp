#pragma once
#include <Eigen/Geometry>
#include <chrono>
#include <iomanip>
#include <numbers>

#include "Leg.hpp"

class Chassis {
public:
  Chassis(float width, float length, Leg *legs[4]);
  void walk();
  Eigen::Vector2f velocity;
  float angularVelocity;

private:
  float width, chassisLength;
  float upperLegLength, lowerLegLength, shoulderWidth, footRadius;
  float reach;
  Eigen::Vector3f basePos;
  Leg *legs[4];
  std::chrono::high_resolution_clock::time_point start{
      std::chrono::high_resolution_clock::now()};

  Eigen::Vector3f p(Eigen::Vector2f v, float t, float n);
};