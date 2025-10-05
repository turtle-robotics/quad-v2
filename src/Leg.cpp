#include "Leg.hpp"

Leg::Leg(
    float upperLen, float lowerLen, float footRadius, float shoulderOffset)
    : upperLen{upperLen},
      lowerLen{lowerLen},
      footRadius{footRadius},
      shoulderOffset{shoulderOffset}
{
}

void Leg::setPosition(Eigen::Vector3f r)
{
  // constexpr float pi{std::numbers::pi_v<float>};
  // float x = r.x(); // - position.x();
  // float y = r.y(); //- position.y() + shoulderOffset;
  // float z = r.z() - footRadius;

  // float shoulderAngle =
  //     atan2f(z, y) - acosf(shoulderOffset / sqrtf(powf(z, 2) + powf(y, 2)));
  // z = sqrtf(powf(z, 2) + powf(y, 2) - powf(shoulderOffset, 2));
  // float d2 = powf(x, 2) + powf(z, 2);
  // float upperAngle =
  //     atan2f(z, x) - acosf((powf(upperLen, 2) + d2 - powf(lowerLen, 2)) /
  //                          (2.0f * upperLen * sqrtf(d2)));
  // float lowerAngle = acosf((powf(upperLen, 2) + powf(lowerLen, 2) - d2) /
  //                          (2.0f * upperLen * lowerLen));
  // std::cout << shoulderAngle << "," << upperAngle << "," << lowerAngle
  //           << std::endl;
  // shoulder.setAngle(shoulderAngle);
  // upperLeg.setAngle(upperAngle);
  // lowerLeg.setAngle(lowerAngle);
}

void Leg::disable()
{
  // shoulder.disable();
  // upperLeg.disable();
  // lowerLeg.disable();
}