#pragma once
#include <Eigen/Core>
#include <iostream>
#include <numbers>

class Leg
{
public:
  Leg(float upperLen, float lowerLen, float footRadius, float shoulderOffset);
  void setPosition(Eigen::Vector3f position);
  void disable();

private:
  // Servo shoulder, upperLeg, lowerLeg;
  float upperLen, lowerLen, footRadius, shoulderOffset;
};