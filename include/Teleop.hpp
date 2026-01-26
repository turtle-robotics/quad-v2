
#pragma once
#include <linux/input.h>
#include <string>

class Teleop {
public:
  Teleop() {};
  int init();
  int config(std::string gamepad_path);
  int readGamepad();
  int close();
  void printGamepad();

  double vx, vy, wz;
  bool home_joints, enable_motors;

private:
  std::string gamepad_path;
  int gamepad = -1;
  struct input_event events[8];
  int new_event_count = 0;
  int total_events = 0;
  const int DEADZONE = 9000;
};