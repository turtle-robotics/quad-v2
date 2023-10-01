#include "SSC32U.hpp"

SSC32U::SSC32U(std::string port, speed_t baudRate)
    : port{port}, baudRate{baudRate} {}

void SSC32U::startSerial() {
  serialPort = open(port.c_str(), O_RDWR);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 0;

  // Set in/out baud rate
  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);

  if (tcsetattr(serialPort, TCSANOW, &tty) == -1) {
    std::cerr << "[Serial][ERROR] Failed to set tty settings on " << port
              << std::endl;
    return;
  }

  std::cout << "[SERIAL][INFO] Serial initialized on " << port << std::endl;
}

void SSC32U::writeSerial(std::string msg) {
  if (write(serialPort, msg.c_str(), msg.size()) == -1)
    std::cerr << "[Serial][ERROR] Failed to write on " << port << std::endl;
}

void SSC32U::setPWM(uint32_t channel, uint32_t pulsewidth) {
  std::ostringstream stream;
  stream << "#" << channel << "P" << pulsewidth << "\r";
  writeSerial(stream.str());
}