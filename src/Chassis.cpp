#include "Chassis.hpp"

Chassis::Chassis(float width, float length, Leg* legs[4]):
width{width},
chassisLength{length}
{
  legs = legs;
  // chassisWidth{chassisWidth},
  //     chassisLength{chassisLength},
  //     LF{LF},
  //     RF{RF},
  //     LB{LB},
  //     RB{RB},
  //     reach{sqrtf(powf(upperLegLength + lowerLegLength, 2) +
  //                 powf(shoulderWidth, 2))},
  //     basePos{0, shoulderWidth - 0.01f, 0.7f * reach}
} // Wider gait for higher stability

// Chassis::~Chassis() {
//   LF.disable();
//   RF.disable();
//   LB.disable();
//   RB.disable();
// }

// Eigen::Vector3f Chassis::p(Eigen::Vector2f v, float t, float n)
// {
//   using std::numbers::pi;
//   if (t < 1.0f)
//   {
//     t += 1.0f;
//     float x = cosf(pi * t) * (1.0f - sinf(pi * t)) + 1.0f;
//     return -0.5f * Eigen::Vector3f{v.x() * x, v.y() * x, powf(sinf(pi * t), 2)};
//   }
//   else
//   {
//     float x = (n - t) / (n - 1.0f);
//     return -Eigen::Vector3f{v.x() * x, v.y() * x, 0.0f};
//   }
// }

// void Chassis::walk()
// {
//   using namespace std::chrono;
//   auto end{high_resolution_clock::now()};
//   duration<float> d = end - start;
//   float n = 2.0; // seconds/cycle

//   float t = d.count();       // seconds
//   float strideLength = 0.1f; // meters

//   Eigen::Vector3f p1 = basePos + strideLength * p(velocity, fmod(t, n), n);
//   Eigen::Vector3f p2 =
//       basePos + strideLength * p(velocity, fmod(t + .25 * n, n), n);
//   Eigen::Vector3f p3 =
//       basePos + strideLength * p(velocity, fmod(t + .5 * n, n), n);
//   Eigen::Vector3f p4 =
//       basePos + strideLength * p(velocity, fmod(t + .75 * n, n), n);

//   if (fabs(velocity.x()) < 0.01 && fabs(velocity.y()) < 0.01)
//   {
//     p1 = basePos;
//     p2 = basePos;
//     p3 = basePos;
//     p4 = basePos;
//   }
//   RF.setPosition(p1);
//   LB.setPosition(p2);
//   LF.setPosition(p3);
//   RB.setPosition(p4);
// }