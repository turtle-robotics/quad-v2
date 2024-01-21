#pragma once
#include <cmath>
#include <map>
#include <numbers>

#include "SSC32U.hpp"

class Servo {
 public:
  Servo(uint32_t channel, SSC32U pwmDriver, uint32_t minPulse,
        uint32_t maxPulse, float minAngle = 0.0f,
        float maxAngle = 2.0f * std::numbers::pi_v<float>);
  void setAngle(float angle);
  void disable();

 private:
  const uint32_t channel;
  SSC32U pwmDriver;
  const uint32_t minPulse, maxPulse;
  const float minAngle, maxAngle;

  uint32_t angleToPWM(float angle);
};