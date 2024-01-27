#include "Robot.hpp"

using namespace std::chrono_literals;

UDPJoystickSocket joystick{5005};

int main() {
  // LFS.setAngle(0);
  // RFS.setAngle(0);
  // LBS.setAngle(0);
  // RBS.setAngle(0);
  // LFU.setAngle(pi / 4);
  // RFU.setAngle(pi / 4);
  // LBU.setAngle(pi / 4);
  // RBU.setAngle(pi / 4);
  // LFL.setAngle(pi / 2.0);
  // RFL.setAngle(pi / 2.0);
  // LBL.setAngle(pi / 2.0);
  // RBL.setAngle(pi / 2.0);
  std::this_thread::sleep_for(5s);
  // LF.disable();
  // RF.disable();
  // LB.disable();
  // RB.disable();
  return 0;
}