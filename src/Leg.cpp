#include "Leg.hpp"

Leg::Leg(Servo shoulder, Servo upperLeg, Servo lowerLeg,
         Eigen::Vector3f position, float upperLen, float lowerLen,
         float footRadius, float shoulderOffset)
    : shoulder{shoulder},
      upperLeg{upperLeg},
      lowerLeg{lowerLeg},
      position{position},
      upperLen{upperLen},
      lowerLen{lowerLen},
      footRadius{footRadius},
      shoulderOffset{shoulderOffset} {}

void Leg::setPosition(Eigen::Vector3f r) {
  constexpr float pi{std::numbers::pi_v<float>};
  float x = r.x() - position.x();
  float y = r.y() - position.y() + shoulderOffset;
  float z =
      sqrtf(upperLen * upperLen + lowerLen * lowerLen) + r.z() - footRadius;

  float shoulderAngle =
      atan2f(z, y) - acosf(shoulderOffset / sqrtf(z * z + y * y));
  z = sqrtf(z * z + y * y - shoulderOffset * shoulderOffset);
  float d2 = x * x + z * z;
  float upperAngle = pi - atan2f(z, x) -
                     acosf((upperLen * upperLen + d2 - lowerLen * lowerLen) /
                           (2.0f * upperLen * sqrtf(d2)));
  float lowerAngle =
      pi - acosf((upperLen * upperLen + lowerLen * lowerLen - d2) /
                 (2.0f * upperLen * lowerLen));

  shoulder.setAngle(shoulderAngle);
  upperLeg.setAngle(upperAngle);
  lowerLeg.setAngle(lowerAngle);
}

void Leg::disable() {
  shoulder.disable();
  upperLeg.disable();
  lowerLeg.disable();
}