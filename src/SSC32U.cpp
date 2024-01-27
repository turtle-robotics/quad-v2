#include "SSC32U.hpp"

SSC32U::SSC32U(std::string port, speed_t baudRate)
    : port{port}, baudRate{baudRate} {
  serialThread = std::jthread{&SSC32U::writeLoop, this};
}

bool SSC32U::startSerial() {
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
    return false;
  }

  std::cout << "[SERIAL][INFO] Serial initialized on " << port << std::endl;
  return true;
}

void SSC32U::writeSerial(std::string msg) {
  if (write(serialPort, msg.c_str(), msg.size()) == -1)
    std::cerr << "[Serial][ERROR] Failed to write on " << port << std::endl;
}

void SSC32U::setPWM(uint32_t channel, uint32_t pulsewidth) {
  outStream += "#" + std::to_string(channel) + "P" + std::to_string(pulsewidth);
}

void SSC32U::writeLoop(std::stop_token stopToken) {
  std::string buffer;

  while (!stopToken.stop_requested()) {
    if (!startSerial()) {
      std::this_thread::sleep_for(5ms);
      return;
    }

    while (!stopToken.stop_requested()) {
      buffer = outStream + "\r";
      outStream = "";
      writeSerial(buffer);
      std::cout << buffer << std::endl;
      std::this_thread::sleep_for(5ms);
    }
  }
}