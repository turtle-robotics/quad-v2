#include "Chassis.hpp"

Chassis::Chassis(float chassisWidth, float chassisLength, float upperLegLength,
                 float lowerLegLength, float shoulderWidth, float footRadius,

                 Servo LFS, Servo LFU, Servo LFL,

                 Servo RFS, Servo RFU, Servo RFL,

                 Servo LBS, Servo LBU, Servo LBL,

                 Servo RBS, Servo RBU, Servo RBL)
    : chassisWidth{chassisWidth},
      chassisLength{chassisLength},
      upperLegLength{upperLegLength},
      lowerLegLength{lowerLegLength},
      shoulderWidth{shoulderWidth},
      footRadius{footRadius},
      LF{LFS,        LFU,          LFL, upperLegLength, lowerLegLength,
         footRadius, shoulderWidth},
      RF{RFS,        RFU,          RFL, upperLegLength, lowerLegLength,
         footRadius, shoulderWidth},
      LB{LBS,        LBU,          LBL, upperLegLength, lowerLegLength,
         footRadius, shoulderWidth},
      RB{RBS,        RBU,          RBL, upperLegLength, lowerLegLength,
         footRadius, shoulderWidth},
      reach{sqrtf(powf(upperLegLength + lowerLegLength, 2) +
                  powf(shoulderWidth, 2))},
      basePos{0, shoulderWidth, 0.8f * reach} {}

Eigen::Vector3f Chassis::p(Eigen::Vector2f v, float t, float n) {
  using std::numbers::pi;
  if (t < 1.0f) {
    t += 1.0f;
    float x = cosf(pi * t) * (1.0f - sinf(pi * t)) + 1.0f;
    return -0.5f * Eigen::Vector3f{v.x() * x, v.y() * x, powf(sinf(pi * t), 2)};
  } else {
    float x = (n - t) / (n - 1.0f);
    return -Eigen::Vector3f{v.x() * x, v.y() * x, 0.0f};
  }
}

void Chassis::walk() {
  using namespace std::chrono;
  auto end{high_resolution_clock::now()};
  duration<float> d = end - start;
  float n = 2.0;

  float t = d.count();
  float strideLength = 0.1f;
  RF.setPosition(basePos + strideLength * p(velocity, t, n));
  LB.setPosition(basePos + strideLength * p(velocity, fmod(t + .75 * n, n), n));
  //   std::cout << out << std::endl;
}