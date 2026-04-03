#include "Leg.hpp"
#include <ModernRobotics>

// Forward Kinematics
void Leg::fk() {
  double th1 = thetalist[0];
  double th2 = thetalist[1];
  double th3 = -thetalist[2] - thetalist[1];
  double h = l[1] * sin(th2) + l[2] * sin(th3);
  double s1 = sin(th1);
  double c1 = cos(th1);

  pf.x() = -l[1] * cos(th2) + l[2] * cos(th3);
  pf.y() = l[0] * c1 - h * s1;
  pf.z() = l[0] * s1 + h * c1 + l[3];
}

// Inverse Kinematics
void Leg::ik() {
  double x = pf.x();
  double y = pf.y();
  double z = pf.z() - l[3];
  double d2;

  thetalist[0] = atan2(z, y) - acos(l[0] / sqrt(y * y + z * z));
  z = sqrt(z * z + y * y -
           l[0] * l[0]); // z projection on plane of upper/lower leg
  d2 = x * x + z * z;
  thetalist[1] = atan2(z, x) - acos((l[1] * l[1] + d2 - l[2] * l[2]) /
                                    (2.0 * l[1] * sqrt(d2)));
  thetalist[2] = -acos((l[1] * l[1] + l[2] * l[2] - d2) / (2.0 * l[1] * l[2]));
}

void Leg::id(const Eigen::Vector3d &g) {
  Eigen::Vector<double, 6> Ftip = Eigen::Vector<double, 6>::Zero(); // N*m, N
  Ftip.tail<3>() = ffoot;
  taulist = mr::InverseDynamics<njoints>(thetalist, dthetalist, ddthetalist, g,
                                         Ftip, Mlist, Glist, Slist);
}

void Leg::run() {
  ik();
  id({0.0, 0.0, 9.81});
}

// void Leg::home() {
//   fmts[0] = fmt_home;
//   fmts[1] = fmt_home;
//   fmts[2] = fmt_home;
//   cmds[0] = PosCmd{};
//   cmds[1] = PosCmd{};
//   cmds[2] = cmd_home;
// }

// void Leg::walk(const Eigen::Vector3d &v, useconds_t dt) {
//   Eigen::Vector3d v_copy = v;
//   v_copy.z() = 0; // ignore vertical velocity for now
//   Eigen::Vector3d dp = Eigen::Vector3d::Zero();
//   double dt_sec = dt * 1e-6; // convert to seconds

//   // Modify state based on position
//   if (v_copy.squaredNorm() < v_min * v_min)
//     state = IDLE;
//   switch (state) {
//   case IDLE: {
//     if (v_copy.squaredNorm() > v_min * v_min) {
//       state = TRAVEL;
//     }
//   } break;
//   case TRAVEL: {
//     if ((p - home_pos).squaredNorm() > travel[1]) {
//       state = LIFT;
//     }
//   } break;
//   }

//   // Calculate change in position based on state
//   switch (state) {
//   case IDLE: {
//     p = home_pos + Eigen::Vector3d(travel * 3 / 5 * legNum - travel, 0, 0);
//   } break;
//   case TRAVEL: {
//     dp = v_copy * dt_sec;
//   } break;
//   case LIFT: {
//     p = lift_pos;
//     // p.z() = lift_height * cos(M_PI_2 * p.head<2>().norm() / travel);
//   } break;
//   case PLACE: {
//     p = -v_copy.normalized() * travel * 0.98 + home_pos;
//   }
//   }

//   p += dp;
//   // t += dt;
//   // p = f();

//   ik(p, theta);
// }

// Eigen::Vector3d Leg::f() {
//   // Given time t that goes from 0 to 1
//   double phase = fmod(t / 1e6, 1.0);

//   // If t is in the first half of the cycle, interpolate from home_pos to
//   // lift_pos
//   if (phase < 0.5) {
//     double alpha = phase / 0.5; // Normalize to [0, 1]
//     return home_pos * (1 - alpha) + lift_pos * alpha;
//   }
//   // If t is in the second half of the cycle, interpolate from lift_pos to
//   // home_pos
//   else {
//     double alpha = (phase - 0.5) / 0.5; // Normalize to [0, 1]
//     return lift_pos * (1 - alpha) + home_pos * alpha;
//   }
// }