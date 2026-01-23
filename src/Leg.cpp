#include "Leg.hpp"

Leg::Leg(
    float lengths[3], float r_foot)
    : l1{lengths[0]},
      l2{lengths[1]},
      l3{lengths[2]},
      footRadius{r_foot}
{
}

void Leg::fk(Eigen::Vector3f theta, Eigen::Vector3f &p)
{
  float th1 = theta[0];
  float th2 = theta[1];
  float th3 = -theta[2] - theta[1];
  float h = l2 * sinf(th2) + l3 * sinf(th3);
  float s1 = sinf(th1);
  float c1 = cosf(th1);

  p.x() = -l2 * cosf(th2) + l3 * cosf(th3);
  p.y() = l1 * c1 - h * s1;
  p.z() = l1 * s1 + h * c1 + footRadius;
}

void Leg::ik(Eigen::Vector3f p, Eigen::Vector3f &theta)
{
  float x = p.x();
  float y = p.y();
  float z = p.z() - footRadius;
  float d2;

  theta[0] = atan2f(z, y) - acosf(l1 / sqrtf(powf(z, 2) + powf(y, 2)));
  z = sqrtf(powf(z, 2) + powf(y, 2) - powf(l1, 2));
  d2 = powf(x, 2) + powf(z, 2);
  theta[1] = atan2f(z, x) - acosf((powf(l2, 2) + d2 - powf(l3, 2)) / (2.0f * l2 * sqrtf(d2)));
  theta[2] = acosf((powf(l2, 2) + powf(l3, 2) - d2) / (2.0f * l2 * l3));
}