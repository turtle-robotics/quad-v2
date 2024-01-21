#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

constexpr uint32_t maxline = 1024;

class UDPJoystickSocket {
 public:
  UDPJoystickSocket(uint16_t port);

 private:
  uint16_t port;
  int sockfd;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  std::jthread udpThread;
  bool startUDP();
  void readLoop(std::stop_token stopToken);
};