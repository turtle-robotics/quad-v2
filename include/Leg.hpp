#pragma once
#include <Eigen/Core>
#include <iostream>
#include <numbers>

#include "Servo.hpp"

class Leg {
 public:
  Leg(Servo shoulder, Servo upperLeg, Servo lowerLeg, Eigen::Vector3f position,
      float upperLen, float lowerLen, float footRadius,
      float shoulderOffset = 0.0f);
  void setPosition(Eigen::Vector3f position);
  void disable();

 private:
  Servo shoulder, upperLeg, lowerLeg;
  Eigen::Vector3f position;
  float upperLen, lowerLen, footRadius, shoulderOffset;
};