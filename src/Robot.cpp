#include "Robot.hpp"

#include <iostream>

// variables prefixed with c_ are YAML nodes
int Robot::configure(YAML::Node conf, bool configure_motors,
                     bool write_motor_config) {
  using Transport = pi3hat::Pi3HatMoteusTransport;
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

  lower_min = c_leg["lowerMin"].as<double>();
  lower_max = c_leg["lowerMax"].as<double>();

  // Create chassis
  const auto &c_chassis = conf["chassis"];
  if (c_chassis["width"].IsNull() || c_chassis["length"].IsNull()) {
    std::cerr << "Invalid configuration file: missing chassis parameters."
              << std::endl;

    return -1;
  }
  chassis_width = c_chassis["width"].as<double>();
  chassis_length = c_chassis["length"].as<double>();

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

void Robot::loop(unsigned int us) {
  prev_state = state;
  // Change State
  switch (state) {
  case IDLE: {
    if (teleop.home_joints) {
      state = HOMING;
    } else if (teleop.deploy_legs) {
      if (legs_deployed) {
        state = DEPLOY_C;
      } else {
        state = DEPLOY_A;
      }
    }
  } break;
  case HOMING: {
    if (!teleop.home_joints) {
      state = IDLE;
    }
  } break;
  case DEPLOY_A:
  case DEPLOY_B:
  case DEPLOY_C: {
    if (!teleop.deploy_legs) {
      state = IDLE;
    }
  } break;
  case RUNNING: {
    // Homing not allowed while running
    if (teleop.home_joints) {
      teleop.home_joints = false;
    }
    if (teleop.deploy_legs) {
      if (legs_deployed) {
        state = DEPLOY_C;
      } else {
        state = DEPLOY_A;
      }
    }
  } break;
  }
  // Act on State
  switch (state) {
  case IDLE: {
    stopMotors();
  } break;
  case HOMING: {
    if (homeMotors() == 1) {
      teleop.home_joints = false;
      state = IDLE;
    }
  } break;
  case DEPLOY_A: {
    if (gotoPose(deploy_a_cmds, deploy_torque) == 1) {
      if (legs_deployed) {
        teleop.deploy_legs = false;
        legs_deployed = false;
        state = IDLE;
      } else {
        state = DEPLOY_B;
      }
    }
  } break;
  case DEPLOY_B: {
    if (gotoPose(deploy_b_cmds, deploy_torque) == 1) {
      if (legs_deployed) {
        state = DEPLOY_A;
      } else {
        state = DEPLOY_C;
      }
    }
  } break;
  case DEPLOY_C: {
    if (gotoPose(deploy_c_cmds, deploy_torque) == 1) {
      if (legs_deployed) {
        state = DEPLOY_B;
      } else {
        state = RUNNING;
        teleop.deploy_legs = false;
        legs_deployed = true;
      }
    }
  } break;
  case RUNNING: {
    gotoPose(stand_cmds);
  } break;
  }

  if (teleop.readGamepad() != 0) {
    std::cerr << "Error reading gamepad input." << std::endl;
  } else {
    // teleop.printGamepad();
  }
  queryMotors();
  printStatus();
}

int Robot::homeMotors() {
  motorState[13] = motors[13]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  motorState[23] = motors[23]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  motorState[33] = motors[33]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  motorState[43] = motors[43]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  if (prev_state != HOMING ||
      (motorState[13].mode == moteus::Mode::kPosition &&
       motorState[13].fault != 102) ||
      (motorState[23].mode == moteus::Mode::kPosition &&
       motorState[23].fault != 102) ||
      (motorState[33].mode == moteus::Mode::kPosition &&
       motorState[33].fault != 102) ||
      (motorState[43].mode == moteus::Mode::kPosition &&
       motorState[43].fault != 102))
    return 0;
  stopMotors();
  ::usleep(1000000); // wait for motors to settle
  // Set home positions for shoulder and upper leg joints
  motors[11]->DiagnosticCommand("d cfg-set-output 0.25");
  motors[21]->DiagnosticCommand("d cfg-set-output 0.25");
  motors[31]->DiagnosticCommand("d cfg-set-output 0.25");
  motors[41]->DiagnosticCommand("d cfg-set-output 0.25");
  motors[12]->DiagnosticCommand("d cfg-set-output 0.0");
  motors[22]->DiagnosticCommand("d cfg-set-output 0.0");
  motors[32]->DiagnosticCommand("d cfg-set-output 0.0");
  motors[42]->DiagnosticCommand("d cfg-set-output 0.0");
  std::ostringstream stringStream;
  stringStream << "Hello";
  std::string copyOfStr = stringStream.str();

  ::sprintf(lower_str, "d cfg-set-output %.4f", lower_min);
  motors[13]->DiagnosticCommand(lower_str);
  motors[23]->DiagnosticCommand(lower_str);
  motors[33]->DiagnosticCommand(lower_str);
  motors[43]->DiagnosticCommand(lower_str);
  return 1;
}

int Robot::gotoPose(std::map<int, double> jointPose, double max_torque) {
  bool all_reached = true;
  for (auto &pose_pair : jointPose) {
    int id = pose_pair.first;
    double position = pose_pair.second;
    moteus::Controller *motor = motors[id];

    PosFmt pos_fmt{
        .position = moteus::kFloat,
        .velocity = moteus::kFloat,
        .maximum_torque = moteus::kFloat,
        .velocity_limit = moteus::kFloat,
    };
    PosCmd cmd;
    cmd.position = position;
    cmd.velocity = NaN;
    cmd.velocity_limit = 1.0;
    cmd.maximum_torque = max_torque;

    const auto state = motor->SetPosition(cmd, &pos_fmt);
    motorState[id] = state->values;
    if (std::abs(state->values.position - position) > 0.01) {
      all_reached = false;
    }
  }
  if (all_reached) {
    return 1;
  }

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

void Robot::printStatus() {
  ::printf("\033[2KState: %s\n", state_names[state].c_str());

  for (auto &state_pair : motorState) {
    int id = state_pair.first;
    const auto &r = state_pair.second;
    if (isnanl(r.position)) {
      ::printf("\033[2K%2d: No data\n", id);
      continue;
    }
    ::printf(
        "\033[2K%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)\n",
        id, static_cast<int>(r.mode), r.position, r.velocity, r.torque,
        r.voltage, r.temperature, r.fault);
  }
  ::printf("\033[%dA", motorState.size() + 1);
  ::fflush(stdout);
}
