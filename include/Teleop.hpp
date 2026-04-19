#pragma once

#include <Eigen/Dense>
#include <linux/input.h>
#include <string>

constexpr double js_norm = 1.0 / 32768.0;

class Teleop {
public:
  Teleop(std::string gamepad_path)
      : gamepad_path{gamepad_path} {
          // std::cout << "Configured gamepad at " << gamepad_path << std::endl;
        };
  int init();
  int readGamepad();

  bool error = false;

  Eigen::Vector<double, 6> V =
      Eigen::Vector<double, 6>::Zero(); // [wx, wy, wz, vx, vy, vz] rad/s, m/s
  bool home_joints, deploy_legs;

private:
  const std::string gamepad_path;
  int gamepad = -1;
  struct input_event events[8];
  int new_event_count = 0;
  int total_events = 0;
  const int deadzone = 400;
};