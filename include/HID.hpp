/**
 * @file HID.hpp
 * @brief HID class for joystick inputs
 */

#pragma once

#include <Eigen/Dense>
#include <linux/input.h>
#include <string>

constexpr double js_norm = 1.0 / 32768.0;

class HID {
public:
  HID(std::string gamepad_path)
      : gamepad_path{gamepad_path} {
          // std::cout << "Configured gamepad at " << gamepad_path << std::endl;
        };
  int init();
  int readGamepad();

  bool error = false;

  /// Joystick twist [wx, wy, wz, vx, vy, vz] rad/s, m/s
  Eigen::Vector<double, 6> V = Eigen::Vector<double, 6>::Zero();

  bool home_joints;
  bool deploy_legs;

private:
  const std::string gamepad_path;
  int gamepad = -1;
  struct input_event events[8];
  int new_event_count = 0;
  int total_events = 0;
  const int deadzone = 400;
};