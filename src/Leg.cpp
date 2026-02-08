#include "Leg.hpp"

Leg::Leg(double l1, double l2, double l3, double r_foot)
    : l1{l1}, l2{l2}, l3{l3}, footRadius{r_foot} {}

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

  theta[0] = atan2(z, y) - acos(l1 / sqrt(y * y + z * z));
  z = sqrt(z * z + y * y - l1 * l1); // z projection on plane of upper/lower leg
  d2 = x * x + z * z;
  theta[1] =
      atan2(z, x) - acos((l2 * l2 + d2 - l3 * l3) / (2.0 * l2 * sqrt(d2)));
  theta[2] = acos((l2 * l2 + l3 * l3 - d2) / (2.0 * l2 * l3));
}

void Leg::fd(Eigen::Vector3d theta, Eigen::Vector3d theta_d,
             Eigen::Vector3d &p_d) {}

void Leg::walk(Eigen::Vector2d v, useconds_t dt) {
  Eigen::Vector2d dp = Eigen::Vector2d::Zero();
  double dt_sec = dt * 1e-6; // convert to seconds

  // Modify state based on position
  switch (state) {
  case IDLE: {
    if (v.squaredNorm() > v_min * v_min) {
      state = TRAVEL;
    }
  } break;
  case TRAVEL: {
    if (p.head<2>().squaredNorm() >= travel2) {
      state = LIFT;
    }
  } break;
  }

  // Calculate change in position based on state
  switch (state) {
  case IDLE: {
    p = home_pos;
  } break;
  case TRAVEL: {
    dp = v * dt_sec;
  } break;
  case LIFT: {
    p = lift_pos;
    // p.z() = lift_height * cos(M_PI_2 * p.head<2>().norm() / travel);
  } break;
  case PLACE: {
    p.head<2>() = -v.normalized() * travel + home_pos.head<2>();
    p.z() = home_pos.z();
  }
  }

  p.head<2>() += dp;

  ik(p, theta);
}