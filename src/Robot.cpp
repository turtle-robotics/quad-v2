// #include <chrono>
// #include <iostream>
// #include <thread>

#include "Config.hpp"
#include "UDPSocket.hpp"

using namespace std::chrono_literals;

UDPJoystickSocket joystick{5005};

int main() {
  while (true) {
    // LFS.setAngle(-pi / 6.0);
    // RFS.setAngle(-pi / 6.0);
    // std::this_thread::sleep_for(1s);
    LFS.setAngle(0);
    RFS.setAngle(0);
    LBS.setAngle(0);
    RBS.setAngle(0);
    LFU.setAngle(0);
    RFU.setAngle(0);
    LBU.setAngle(0);
    RBU.setAngle(0);
    LFL.setAngle(pi / 2.0);
    RFL.setAngle(pi / 2.0);
    LBL.setAngle(pi / 2.0);
    RBL.setAngle(pi / 2.0);
    std::this_thread::sleep_for(1s);
  }
  return 0;
}