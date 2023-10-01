#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>

#include "SSC32U.hpp"

using namespace std::chrono_literals;

SSC32U pwmDriver{"/dev/ttyUSB0", B9600};

int main() {
  std::this_thread::sleep_for(1s);
  pwmDriver.startSerial();
  Eigen::Quaternion a{0, 1, 1, 1};
  Eigen::Quaternion b{0, -1, 1, 1};
  std::cout << a * b << std::endl;
  pwmDriver.setPWM(0, 1000);
  std::this_thread::sleep_for(1s);
  pwmDriver.setPWM(0, 2000);
  return 0;
}