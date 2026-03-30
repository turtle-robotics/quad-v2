#include "Robot.hpp"
#include "ConfigLoader.hpp"

// variables prefixed with c_ are YAML nodes
int Robot::configure(YAML::Node conf, bool configure_motors,
                     bool write_motor_config) {

  if (configured) {
    std::cerr << "Robot is already configured." << std::endl;
    return -1;
  }

  std::cout << "Configuring robot..." << std::endl;

  // Create chassis
  const auto &c_chassis = conf["chassis"];
  if (c_chassis["width"].IsNull() || c_chassis["length"].IsNull()) {
    std::cerr << "Invalid configuration file: missing chassis parameters."
              << std::endl;

    return -1;
  }
  chassis_width = c_chassis["width"].as<double>();
  chassis_length = c_chassis["length"].as<double>();

#ifdef RPI
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
  ::sprintf(lower_str, "d cfg-set-output %.4f",
            conf["leg"]["lower"]["lim"][0].as<double>());
  if (configure_motors) {
    std::cout << "Writing motor configuration..." << std::endl;

    std::string conf_str;
    const auto &c_motor_conf = conf["motor_config"];
    for (auto &c_line : c_motor_conf)
      for (auto &motor_pair : motors) {
        conf_str = "conf set " + c_line.as<std::string>();
        motor_pair.second->DiagnosticCommand(conf_str);
        ::printf("Wrote config \"%s\" to motor %d\n", conf_str.c_str(),
                 motor_pair.first);
      }
    const auto &c_motor_dir = conf["motor_directions"];
    for (auto &c_motor : c_motor_dir) {
      int can_id = c_motor.first.as<int>();
      int direction = c_motor.second.as<int>();
      if (motors.find(can_id) != motors.end()) {
        conf_str =
            "conf set motor_position.output.sign " + std::to_string(direction);
        motors[can_id]->DiagnosticCommand(conf_str);
        ::printf("Wrote config \"%s\" to motor %d\n", conf_str.c_str(), can_id);
      } else {
        std::cerr << "Invalid configuration file: motor_directions contains "
                  << "invalid can_id " << can_id << "." << std::endl;
        return -1;
      }
    }
  }
  if (write_motor_config) {
    std::cout << "Writing motor configuration to flash..." << std::endl;
    for (auto &motor_pair : motors)
      motor_pair.second->DiagnosticCommand("conf write");
  }
#endif

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

#ifdef RPI
  // Setup each motor
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;
    const auto state = motor->SetStop();
    *motorState[id] = state->values;
    // moteus::GpioWrite::Command gpio_cmd;
    // gpio_cmd.aux1 = 1; // set aux1 to high
    // motor->SetWriteGpio(gpio_cmd);
  }

  // Initialize motors
  stopMotors();
#endif

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
    if (gotoJointPose(deploy_a_cmds, deploy_torque) == 1) {
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
    if (gotoJointPose(deploy_b_cmds, deploy_torque) == 1) {
      if (legs_deployed) {
        state = DEPLOY_A;
      } else {
        state = DEPLOY_C;
      }
    }
  } break;
  case DEPLOY_C: {
    if (gotoJointPose(deploy_c_cmds, deploy_torque) == 1) {
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
    int leg_id = 0;
    for (auto &leg : legs) {
      leg_id++;
      Eigen::Vector3d v = 0.2 * teleop.V.tail<3>(); // body velocity in x, y, z
      // leg->walk(v, us);
      std::map<int, double> jointAngles{
          {leg_id * 10 + 1, leg->thetalist[0] * 0.5 * M_1_PI},
          {leg_id * 10 + 2, leg->thetalist[1] * 0.5 * M_1_PI},
          {leg_id * 10 + 3, leg->thetalist[2] * 0.5 * M_1_PI},
      };
      if (gotoJointPose(jointAngles) == 1) {
        if (leg->state == Leg::LIFT) {
          leg->state = Leg::PLACE;
        } else if (leg->state == Leg::PLACE) {
          leg->state = Leg::RUNNING;
        }
      }
    }
  } break;
  }
  if (state != RUNNING && prev_state == RUNNING) {
    for (auto &leg : legs) {
      leg->state = Leg::IDLE;
    }
  }

  teleop.readGamepad();
  queryMotors();
  printStatus();
}

int Robot::homeMotors() {
#ifdef RPI
  *motorState[13] =
      motors[13]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  *motorState[23] =
      motors[23]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  *motorState[33] =
      motors[33]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  *motorState[43] =
      motors[43]->SetPosition(homing_cmd, &homing_pos_fmt)->values;
  if (prev_state != HOMING ||
      (motorState[13]->mode == moteus::Mode::kPosition &&
       motorState[13]->fault != 102) ||
      (motorState[23]->mode == moteus::Mode::kPosition &&
       motorState[23]->fault != 102) ||
      (motorState[33]->mode == moteus::Mode::kPosition &&
       motorState[33]->fault != 102) ||
      (motorState[43]->mode == moteus::Mode::kPosition &&
       motorState[43]->fault != 102))
    return 0;
#else
  std::cout << "Homing Motors" << std::endl;
#endif
  stopMotors();
  ::usleep(1000000); // wait for motors to settle

  // Set home positions for shoulder and upper leg joints
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    if (id % 10 == 1) {
      if ((id / 10) % 2 == 0) {
        motor_pair.second->DiagnosticCommand("d cfg-set-output 0.25");
      } else {
        motor_pair.second->DiagnosticCommand("d cfg-set-output -0.25");
      }
    } else if (id % 10 == 2) {
      motor_pair.second->DiagnosticCommand("d cfg-set-output 0.0");
    } else if (id % 10 == 3) {
      motor_pair.second->DiagnosticCommand(lower_str);
    }
  }
  return 1;
}

int Robot::gotoCartesianPose(const std::map<int, Eigen::Translation3d> &legPose,
                             double max_torque) {
  std::map<int, double> jointPose;
  for (auto &pose_pair : legPose) {
    int leg_id = pose_pair.first;
    legs[leg_id + 1]->pf = pose_pair.second;
    legs[leg_id + 1]->ik();
    // Degrees to turns
    jointPose[leg_id * 10 + 1] = legs[leg_id + 1]->thetalist[0] * 0.5 * M_1_PI;
    jointPose[leg_id * 10 + 2] = legs[leg_id + 1]->thetalist[1] * 0.5 * M_1_PI;
    jointPose[leg_id * 10 + 3] = legs[leg_id + 1]->thetalist[2] * 0.5 * M_1_PI;
  }

  // return gotoJointPose(jointPose, max_torque);
  return 0;
}

int Robot::gotoJointPose(const std::map<int, double> &jointPose,
                         double max_torque) {
#ifdef RPI
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
    *motorState[id] = state->values;
    if (std::abs(state->values.position - position) > 0.01) {
      all_reached = false;
    }
  }
  if (all_reached) {
    return 1;
  }
#else
  std::cout << "Goint to joint pose " << jointPose[0].first << ": "
            << jointPose[0].second << ", " << jointPose[1].first << ": "
            << jointPose[1].second << ", " << jointPose[2].first << ": "
            << jointPose[2].second;
#endif
  return 0;
}

void Robot::queryMotors() {
#ifdef RPI
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    auto result = motor->SetQuery();
    if (!result.has_value()) {
      *motorState[id] = moteus::Query::Result{};
      continue;
    }
    *motorState[id] = result->values;
  }
#endif
}

void Robot::stopMotors() {
#ifdef RPI
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    const auto state = motor->SetStop();
    *motorState[id] = state->values;
  }
#endif
}

void Robot::holdPosition() {
#ifdef RPI
  for (auto &motor_pair : motors) {
    int id = motor_pair.first;
    moteus::Controller *motor = motor_pair.second;

    const auto state = motor->SetZeroVelocity();
    *motorState[id] = state->values;
  }
#endif
}

void Robot::printStatus() {
  ::printf("\033[2KState: %s\n", state_names[state].c_str());
  if (teleop.error) {
    ::printf("\033[2KGamepad: Not Connected\n");
  } else {
    ::printf("\033[2KGamepad: w=(%6.3f,%6.3f,%6.3f) v=(%6.3f,%6.3f,%6.3f) "
             "home=%1d deploy=%1d\n",
             teleop.V[0], teleop.V[1], teleop.V[2], teleop.V[3], teleop.V[4],
             teleop.V[5], teleop.home_joints, teleop.deploy_legs);
  }

  for (auto &state_pair : motorState) {
    int id = state_pair.first;
    const auto &r = state_pair.second;
    if (isnanl(r->position)) {
      ::printf("\033[2K%2d: No data\n", id);
      continue;
    }
    ::printf("\033[2K%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  "
             "v/t/f=(%5.1f,%5.1f,%3d)\n",
             id, static_cast<int>(r->mode), r->position, r->velocity, r->torque,
             r->voltage, r->temperature, r->fault);
  }
  int leg_id = 0;
  for (auto &leg : legs) {
    leg_id++;
    ::printf("\033[2KLeg %d: state=%s p=(%6.3f,%6.3f,%6.3f) "
             "theta=(%6.3f,%6.3f,%6.3f)\n",
             leg_id,
             leg->state == Leg::IDLE      ? "IDLE   "
             : leg->state == Leg::LIFT    ? "LIFT   "
             : leg->state == Leg::PLACE   ? "PLACE  "
             : leg->state == Leg::RUNNING ? "RUNNING"
                                          : "UNKNOWN",
             leg->pf.x(), leg->pf.y(), leg->pf.z(), leg->thetalist[0],
             leg->thetalist[1], leg->thetalist[2]);
  }
  ::printf("\033[%dA", motorState.size() + legs.size() + 2);
  ::fflush(stdout);
}
