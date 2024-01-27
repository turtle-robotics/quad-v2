#include "Servo.hpp"

Servo::Servo(uint32_t channel, SSC32U& pwmDriver, uint32_t minPulse,
             uint32_t maxPulse, float minAngle, float maxAngle)
    : channel{channel},
      pwmDriver{pwmDriver},
      minPulse{maxPulse > minPulse ? minPulse : maxPulse},
      maxPulse{maxPulse > minPulse ? maxPulse : minPulse},
      minAngle{maxPulse > minPulse ? minAngle : maxAngle},
      maxAngle{maxPulse > minPulse ? maxAngle : minAngle} {}

uint32_t Servo::angleToPWM(float angle) {
  float t = (angle - minAngle) / (maxAngle - minAngle);
  if (t < 0.0f) return minPulse;
  if (t > 1.0f) return maxPulse;
  return std::lerp(minPulse, maxPulse, t);
}

void Servo::setAngle(float angle) {
  pwmDriver.setPWM(channel, angleToPWM(angle));
}

void Servo::disable() { pwmDriver.setPWM(channel, 0); }