#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

class SSC32U {
 public:
  SSC32U(std::string port, speed_t baudRate);
  void startSerial();
  void setPWM(uint32_t channel, uint32_t pulsewidth);

 private:
  const std::string port;
  const speed_t baudRate;
  int serialPort;
  struct termios tty;

  void writeSerial(std::string msg);
};