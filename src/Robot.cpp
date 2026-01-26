#include "Robot.hpp"

#include <signal.h>

#include <iostream>

void signal_callback_handler(int signum) {
  ::printf("\033[2J\033[H"); // clear screen
  robot.stopMotors();

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

// Legs are numbered from 1 to 4: Front Right, Front Left, Back Right, Back Left
// Joints are numbered from 1 to 3: Shoulder, Upper Leg, Lower Leg
// Motor CAN IDs follow: AB where A is the leg number and B is the joint number

int main(int argc, char *argv[]) {
  // Handle Ctrl-C signal to stop motors safely
  signal(SIGINT, signal_callback_handler);

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
        if (argv[i][j] == 'w')
          write_motor_config = true;
        if (argv[i][j] == 'c')
          configure_motors = true;
        else {
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

  if (robot.configure(config, configure_motors, write_motor_config) != 0) {
    std::cerr << "Failed to configure robot." << std::endl;
    return 1;
  }

  if (robot.init() != 0) {
    std::cerr << "Failed to initialize robot." << std::endl;
    return 1;
  }

  std::cout << "Running..." << std::endl;
  while (true) {
    robot.loop(10000);
    ::usleep(10000);
  }

  return 0;
}

void Robot::loop(int us) {
  // queryMotors();
  if (teleop.readGamepad() != 0) {
    std::cerr << "Error reading gamepad input." << std::endl;
  } else {
    teleop.printGamepad();
  }
  // gotoZero();
  // printMotorStatus();
}

using Transport = pi3hat::Pi3HatMoteusTransport;

// variables prefixed with c_ are YAML nodes
int Robot::configure(YAML::Node conf, bool configure_motors,
                     bool write_motor_config) {
  if (configured) {
    std::cerr << "Robot is already configured." << std::endl;
    return -1;
  }

  std::cout << "Configuring robot..." << std::endl;

  // Create mjbots pi3hat transport
  Transport::Options toptions;

  const auto &c_servomap = conf["servomap"];
  for (auto &c_bus_id : c_servomap) {
    int bus = c_bus_id.first.as<int>();
    for (auto &c_servo_id : c_bus_id.second) {
      toptions.servo_map[c_servo_id.as<int>()] = bus;
    }
    ::fflush(stdout);
  }
  const auto transport = std::make_shared<Transport>(toptions);

  // Create motors
  for (int leg_id = 1; leg_id <= 4; leg_id++) {
    for (int joint_id = 1; joint_id <= 3; joint_id++) {
      int can_id = leg_id * 10 + joint_id;

      motors[can_id] = new moteus::Controller([&]() {
        moteus::Controller::Options coptions;
        coptions.id = can_id;
        coptions.transport = transport;
        return coptions;
      }());
    }
  }

  // Create legs
  const auto &c_leg = conf["leg"];
  float lengths[3] = {c_leg["upperLen"].as<float>(),
                      c_leg["lowerLen"].as<float>(),
                      c_leg["shoulderLen"].as<float>()};
  float footRadius = c_leg["footRadius"].as<float>();
  // for (int i = 1; i <= 4; i++)
  //     legs[i] = new Leg(lengths, footRadius);

  // Create chassis
  const auto &c_chassis = conf["chassis"];
  if (c_chassis["width"].IsNull() || c_chassis["length"].IsNull()) {
    std::cerr << "Invalid configuration file: missing chassis parameters."
              << std::endl;

    return -1;
  }

  if (configure_motors) {
    std::cout << "Writing motor configuration..." << std::endl;
    const auto &c_motor_conf = conf["motor_config"];
    for (auto &c_line : c_motor_conf)
      for (auto &motor_pair : motors) {
        std::stringstream ss;
        ss << "conf set " << c_line.as<std::string>();
        motor_pair.second->DiagnosticCommand(ss.str());
        std::cout << "Wrote motor " << motor_pair.first
                  << " config: " << ss.str() << std::endl;
      }
  }
  if (write_motor_config) {
    std::cout << "Writing motor configuration to flash..." << std::endl;
    for (auto &motor_pair : motors)
      motor_pair.second->DiagnosticCommand("conf write");
  }
  std::string gamepad_path = conf["gamepad"].as<std::string>();
  teleop.config(gamepad_path);
  std::cout << "Configured gamepad at " << gamepad_path << std::endl;

  std::cout << "Robot configured successfully." << std::endl;
  configured = true;
  return 0;
}

int Robot::init() {
  if (!configured) {
    std::cerr << "Robot is not configured." << std::endl;
    return -1;
  }
  if (initialized) {
    std::cerr << "Robot is already initialized." << std::endl;
    return -1;
  }

  std::cout << "Initializing robot..." << std::endl;

  // Setup each motor
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;
    const auto state = motor->SetStop();
    motorState[id] = state->values;
    moteus::GpioWrite::Command gpio_cmd;
    gpio_cmd.aux1 = 1; // set aux1 to high
    motor->SetWriteGpio(gpio_cmd);
  }

  // Initialize motors
  stopMotors();

  // Initialize controller
  std::cout << "Initializing controller..." << std::endl;
  if (teleop.init() != 0) {
    std::cerr << "Failed to initialize teleop." << std::endl;
    return -1;
  }

  ::usleep(10000); // small delay

  std::cout << "Robot initialized successfully." << std::endl;
  initialized = true;
  return 0;
}

void Robot::queryMotors() {
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    auto result = motor->SetQuery();
    if (!result.has_value()) {
      motorState[id] = moteus::Query::Result{};
      continue;
    }
    motorState[id] = result->values;
  }
}

void Robot::stopMotors() {
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    const auto state = motor->SetStop();
    motorState[id] = state->values;
  }
}

void Robot::holdPosition() {
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    const auto state = motor->SetZeroVelocity();
    motorState[id] = state->values;
  }
}

void Robot::gotoZero() {
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    if (id < 20 || id > 23 && (id < 40 || id > 43))
      continue;
    moteus::Controller *motor = motor_pair.second;

    moteus::PositionMode::Command cmd;
    cmd.position = 0.0;
    cmd.velocity = NaN;
    cmd.feedforward_torque = 0.0;

    const auto state = motor->SetPosition(cmd);
    motorState[id] = state->values;
  }
}

void Robot::printMotorStatus() {
  for (auto &state_pair : motorState) {
    int id = state_pair.first;
    const auto &r = state_pair.second;
    if (isnanl(r.position)) {
      ::printf(
          "%2d:  No data                                                 \n",
          id);
      continue;
    }
    ::printf("%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \n",
             id, static_cast<int>(r.mode), r.position, r.velocity, r.torque,
             r.voltage, r.temperature, r.fault);
  }
  ::printf("\033[%dA", motorState.size());
  ::fflush(stdout);
}
