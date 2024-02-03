#include "Robot.hpp"

#include <Eigen/Dense>
#include <iostream>

using namespace std::chrono_literals;

UDPJoystickSocket joystick{5005, chassis.velocity};

// float d_max =
//     sqrt(pow(upperLegLength + lowerLegLength, 2) + pow(shoulderWidth, 2));
// float l_stride = 0.1;
// Eigen::Vector3f basePos{0, shoulderWidth, 0.8f * d_max};

// Eigen::Vector3f p(float t) {
//   float n = 2;
//   Eigen::Vector3f a;
//   if (t < 1.0f) {
//     t += 1.0f;
//     a = Eigen::Vector3f{cosf(pi * t) * (1.0f - sinf(pi * t)) + 1.0f, 0.0f,
//                         powf(sinf(pi * t), 2)};
//     a *= 0.5f;
//   } else
//     a = Eigen::Vector3f{(n - t) / (n - 1.0f), 0.0f, 0.0f};
//   // std::cout << a << std::endl;
//   return basePos - l_stride * a;
// }

int main() {
  std::ios_base::sync_with_stdio(false);
  while (true) {
    chassis.walk();
    std::this_thread::sleep_for(10ms);
  }
  // for (float t = 0.0f; t <= 2.0f; t += 0.01f) {
  //   // RF.setPosition(p(t));
  //   // LB.setPosition(p(fmod(t + .75 * 2.0, 2)));
  //   chassis.walk();
  //   std::this_thread::sleep_for(10ms);
  // }
  // LF.disable();
  // RF.disable();
  // LB.disable();
  // RB.disable();
  return 0;
}