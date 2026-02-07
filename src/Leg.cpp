#include "Leg.hpp"

Leg::Leg(double lengths[3], double r_foot)
    : l1{lengths[0]}, l2{lengths[1]}, l3{lengths[2]}, footRadius{r_foot} {}

void Leg::fk(Eigen::Vector3d theta, Eigen::Vector3d &p) {
  double th1 = theta[0];
  double th2 = theta[1];
  double th3 = -theta[2] - theta[1];
  double h = l2 * sin(th2) + l3 * sin(th3);
  double s1 = sin(th1);
  double c1 = cos(th1);

  p.x() = -l2 * cos(th2) + l3 * cos(th3);
  p.y() = l1 * c1 - h * s1;
  p.z() = l1 * s1 + h * c1 + footRadius;
}

void Leg::ik(Eigen::Vector3d p, Eigen::Vector3d &theta) {
  double x = p.x();
  double y = p.y();
  double z = p.z() - footRadius;
  double d2;

  theta[0] = atan2(z, y) - acos(l1 / sqrt(pow(z, 2) + pow(y, 2)));
  z = sqrt(pow(z, 2) + pow(y, 2) - pow(l1, 2));
  d2 = pow(x, 2) + pow(z, 2);
  theta[1] = atan2(z, x) -
             acos((pow(l2, 2) + d2 - pow(l3, 2)) / (2.0 * l2 * sqrt(d2)));
  theta[2] = acos((pow(l2, 2) + pow(l3, 2) - d2) / (2.0 * l2 * l3));
}

void Leg::fd(Eigen::Vector3d theta, Eigen::Vector3d theta_d,
             Eigen::Vector3d &p_d) {}

void Leg::trot(const Eigen::Vector2d &v, Eigen::Vector3d &p, useconds_t &dt) {
  dp = Eigen::Vector2d::Zero();

  // Modify state based on position
  switch (state) {
  case IDLE: {
    if (v.norm() > 0.01) {
      state = TRAVEL;
    }
  } break;
  case TRAVEL: {
    if (p.head<2>().norm() >= travel) {
      state = LIFT;
    }
  } break;
  case LIFT: {
    if (p.head<2>().norm() >= travel) {
      state = TRAVEL;
      p.z() = 0.0;
    }
  } break;
  }

  // Calculate change in position based on state
  switch (state) {
  case IDLE: {
    p = Eigen::Vector3d::Zero();
  } break;
  case TRAVEL: {
    dp = v * dt;
  } break;
  case LIFT: {
    dp = v_r * v.normalized() * dt;
    p.z() = lift_height * cos(M_PI_2 * p.head<2>().norm() / travel);
  } break;
  }

  p.head<2>() += dp;
}