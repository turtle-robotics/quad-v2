#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

constexpr uint32_t maxline = sizeof(Eigen::Vector2f);

class UDPJoystickSocket {
 public:
  UDPJoystickSocket(uint16_t port, Eigen::Vector2f &data);

 private:
  Eigen::Vector2f &data;
  uint16_t port;
  int sockfd;
  struct sockaddr_in servaddr, cliaddr;
  socklen_t len;
  std::jthread udpThread;
  bool startUDP();
  void readLoop(std::stop_token stopToken);
};