#include <chrono>
#include <iostream>
#include <thread>

#include "Leg.hpp"

using namespace std::chrono_literals;

SSC32U pwmDriver{"/dev/ttyUSB0", B9600};
Servo s1{15, pwmDriver, 0, 2000};
Leg l1{{1, pwmDriver, 0, 2000},
       {2, pwmDriver, 0, 2000},
       {3, pwmDriver, 0, 2000},
       {1, 1, 1},
       1,
       1,
       1};

int main() {
  l1.setPosition({1, 2, 1});
  l1.disable();
  return 0;
}