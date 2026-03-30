#include "ConfigLoader.hpp"
#include "Robot.hpp"
#include <algorithm>
#include <errno.h>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>

constexpr useconds_t duty_cycle_us = 10000;
std::shared_ptr<Robot> robot; // global robot instance

void signal_callback_handler(int signum) {
  ::printf("\033[2J\033[H"); // clear screen
  robot->stopMotors();

  exit(signum);
}

void print_help(const char *program_name) {
  std::cout << "Usage: " << program_name << " <config_file.yaml> [-chw]"
            << std::endl;
  std::cout << "  -c : Configure motors from the configuration file."
            << std::endl;
  std::cout << "  -w : Write motor configuration to flash." << std::endl;
  std::cout << "  -h : Print this help message." << std::endl;
}

void configure_rt() {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    std::cerr << "mlockall failed: " << strerror(errno) << std::endl;
  }
  struct sched_param param{.sched_priority = 99};

  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    std::cerr << "sched_setscheduler failed: " << strerror(errno) << std::endl;
  } else {
    std::cout << "Real-time scheduling enabled with priority "
              << param.sched_priority << std::endl;
  }
}

// Legs are numbered from 1 to 4: Front Right, Front Left, Back Right, Back Left
// Joints are numbered from 1 to 3: Shoulder, Upper Leg, Lower Leg
// Motor CAN IDs follow: AB where A is the leg number and B is the joint number

int main(int argc, char *argv[]) {
  // Handle Ctrl-C signal to stop motors safely
  signal(SIGINT, signal_callback_handler);

  // Configure real-time scheduling
  configure_rt();

  // Load configuration file from command line argument
  if (argc < 2) {
    print_help(argv[0]);
    return 1;
  }

  bool configure_motors = false;
  bool write_motor_config = false;

  for (int i = 2; i < argc; i++) {
    if (argv[i][0] == '-') {
      for (int j = 1; argv[i][j] != '\0'; j++) {
        if (argv[i][j] == 'h') {
          print_help(argv[0]);
          return 0;
        }
        if (argv[i][j] == 'w') {
          write_motor_config = true;
        } else if (argv[i][j] == 'c') {
          configure_motors = true;
        } else {
          std::cerr << "Unknown option: -" << argv[i][j] << std::endl;
          print_help(argv[0]);
          return 1;
        }
      }
    }
  }

  std::string config_file = argv[1];

  // Load and parse the YAML configuration file
  std::cout << "Starting robot with config file " << config_file << std::endl;
  YAML::Node config = YAML::LoadFile(config_file);

  robot = std::make_shared<Robot>(
      config["legs"].as<std::array<std::shared_ptr<Leg>, 4>>());

  if (robot->configure(config, configure_motors, write_motor_config) != 0) {
    std::cerr << "Failed to configure robot." << std::endl;
    return 1;
  }

  if (robot->init() != 0) {
    std::cerr << "Failed to initialize robot." << std::endl;
    return 1;
  }

  std::cout << "Running..." << std::endl;
  while (true) {
    robot->loop(duty_cycle_us);
    ::usleep(duty_cycle_us);
  }

  return 0;
}