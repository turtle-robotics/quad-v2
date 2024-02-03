#include "UDPSocket.hpp"
// https://www.geeksforgeeks.org/udp-server-client-implementation-c/

UDPJoystickSocket::UDPJoystickSocket(uint16_t port, Eigen::Vector2f &data)
    : port{port}, data{data} {
  udpThread = std::jthread{&UDPJoystickSocket::readLoop, this};
}

bool UDPJoystickSocket::startUDP() {
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "[UDP][ERROR] Failed to open socket on port " << port
              << std::endl;
    return false;
  }
  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  servaddr.sin_family = AF_INET;  // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);

  if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "[UDP][ERROR] Failed to bind port " << port << std::endl;
    return false;
  }

  std::cout << "[UDP][INFO] UDP connected on port " << port << std::endl;
  return true;
}

void UDPJoystickSocket::readLoop(std::stop_token stopToken) {
  char buffer[maxline];
  const struct timeval timeout = {
      .tv_sec = 0, .tv_usec = std::chrono::microseconds(5ms).count()};
  struct timeval this_timeout;
  fd_set fds;

  while (!stopToken.stop_requested()) {
    if (!startUDP()) {
      std::this_thread::sleep_for(5ms);
      continue;
    }

    while (!stopToken.stop_requested()) {
      FD_ZERO(&fds);
      FD_SET(sockfd, &fds);
      this_timeout = timeout;
      if (select(sockfd + 1, &fds, NULL, NULL, &this_timeout) < 1 &&
          !FD_ISSET(sockfd, &fds))
        continue;
      int n = recvfrom(sockfd, &data, sizeof(data), MSG_WAITALL,
                       (struct sockaddr *)&cliaddr, &len);
      if (n < 0) {
        std::cerr << "[UDP][ERROR] Failed to receive from UDP" << std::endl;
        return;
      }
      // buffer[n] = '\0';
      std::cout << "Client :" << sizeof(data) << ", " << data.x() << " "
                << data.y() << std::endl;
    }
  }
}