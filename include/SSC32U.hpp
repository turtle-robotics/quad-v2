#pragma once
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

class SSC32U {
 public:
  SSC32U(std::string port, speed_t baudRate);
  void setPWM(uint32_t channel, uint32_t pulsewidth);

 private:
  std::string port;
  speed_t baudRate;
  int serialPort;
  struct termios tty;
  std::string output;
  std::jthread serialThread;
  bool startSerial();
  void writeLoop(std::stop_token stopToken);
  void writeSerial(std::string msg);
};