#include "Teleop.hpp"

#include <fcntl.h>
#include <iostream>
#include <unistd.h>

int Teleop::config(std::string gamepad_path) {
  this->gamepad_path = gamepad_path;
  return 0;
}

int Teleop::init() {
  gamepad = open(gamepad_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (gamepad == -1) {
    std::cerr << "Failed to open gamepad device at " << gamepad_path
              << std::endl;
    return -1;
  }

  return 0;
}

int Teleop::close() { return 0; }

int Teleop::readGamepad() {
  ssize_t r1 = read(gamepad, events, sizeof events);
  if (r1 == -1)
    if (errno == EWOULDBLOCK || errno == EAGAIN) // these are fine
      return 0;
    else
      return -1; // actual error

  new_event_count = r1 / sizeof(struct input_event);

  for (int evi = 0; evi < new_event_count; evi++) {
    auto &ev = events[evi];
    switch (ev.type) {
    case EV_ABS: {
      switch (ev.code) {
      case ABS_X: {
        wz = ev.value;
      } break;
      case ABS_Y: {
        vx = ev.value;
      } break;
      case ABS_RX: {
        vy = ev.value;
      } break;
      }
    } break;
    case EV_KEY: {
      switch (ev.code) {
      case BTN_START: {
        if (ev.value == 1) {
          enable_motors = !enable_motors;
        }
      } break;
      case BTN_SELECT: {
        if (ev.value == 1) {
          home_joints = !home_joints;
        }
      } break;
      case BTN_A: {
      } break;
      case BTN_B: {
      } break;
      }
    } break;
    }
    total_events++;
  }
  return 0;
}

void Teleop::printGamepad() {
  std::cout << "Gamepad State: vx=" << vx << " vy=" << vy << " wz=" << wz
            << " enable_motors=" << enable_motors
            << " home_joints=" << home_joints << std::endl;
}