#include "Robot.hpp"
#include "config_loader.hpp"

// variables prefixed with c_ are YAML nodes
int Robot::configure(YAML::Node conf, bool configure_motors,
                     bool write_motor_config) {

  if (configured) {
    std::cerr << "Robot is already configured." << std::endl;
    return -1;
  }

  std::cout << "Configuring robot..." << std::endl;
#if defined(__aarch64__)
  std::cout << "Configuring real-time" << std::endl;
  mjbots::pi3hat::ConfigureRealtime();
#endif

  // Create motors
  home_joint_pose = conf["home_pose"].as<JointPose>();
  deploy_a_cmds = conf["deploy_cmds"]["a"].as<JointPose>();
  deploy_b_cmds = conf["deploy_cmds"]["b"].as<JointPose>();
  deploy_c_cmds = conf["deploy_cmds"]["c"].as<JointPose>();

  if (configure_motors) {
    std::cout << "Writing motor configuration..." << std::endl;

    std::string conf_str;
    for (const auto &config_line : conf["motor_config"])
      for (const auto &leg_motors : motors)
        for (const auto &motor : leg_motors) {
          conf_str = "conf set " + config_line.as<std::string>();
          motor->DiagnosticCommand(conf_str);
          // TODO: Add to logging functionality
          // ::printf("Wrote config \"%s\" to motor %d\n", conf_str.c_str(),
          //          motor_pair.first);
        }
  }
  if (write_motor_config) {
    std::cout << "Writing motor configuration to flash..." << std::endl;
    for (const auto &leg_motors : motors)
      for (const auto &motor : leg_motors) {
        motor->DiagnosticCommand("conf write");
      }
  }

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

  // moteus::GpioWrite::Command gpio_cmd;
  // gpio_cmd.aux1 = 1; // set aux1 to high
  // motor->SetWriteGpio(gpio_cmd);

  // Initialize motors
  stopMotors();

  // Initialize controller
  std::cout << "Initializing controller..." << std::endl;
  if (teleop->init() != 0) {
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

  // chassis->run();
  // for (const auto leg : legs) {

  // }

  // Change State
  switch (state) {
  case IDLE: {
    if (teleop->home_joints) {
      state = HOMING;
    } else if (teleop->deploy_legs) {
      if (legs_deployed) {
        state = DEPLOY_C;
      } else {
        state = DEPLOY_A;
      }
    }
  } break;
  case HOMING: {
    if (!teleop->home_joints) {
      state = IDLE;
    }
  } break;
  case DEPLOY_A:
  case DEPLOY_B:
  case DEPLOY_C: {
    if (!teleop->deploy_legs) {
      state = IDLE;
    }
  } break;
  case RUNNING: {
    // Homing not allowed while running
    if (teleop->home_joints) {
      teleop->home_joints = false;
    }
    if (teleop->deploy_legs) {
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
      teleop->home_joints = false;
      state = IDLE;
    }
  } break;
  case DEPLOY_A: {
    if (gotoJointPose(deploy_a_cmds, deploy_torque) == 1) {
      if (legs_deployed) {
        teleop->deploy_legs = false;
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
        teleop->deploy_legs = false;
        legs_deployed = true;
      }
    }
  } break;
  case RUNNING: {
    // TODO: Gotta figure this one out
    chassis->Vb = teleop->V;
    chassis->run();
    for (unsigned i = 0; i < 4; i++) {
      legs[i]->run();
      motorPosCmds[i] =
          makePosCmd(legs[i]->thetalist, legs[i]->thetadlist, legs[i]->taulist);
    }

    // int leg_id = 0;
    // for (auto &leg : legs) {
    //   leg_id++;
    //   Eigen::Vector3d v = 0.2 * teleop.V.tail<3>(); // body velocity in x, y,
    //   z
    //   // leg->walk(v, us);
    //   JointPose jointAngles{
    //       {leg_id * 10 + 1, leg->thetalist[0] * 0.5 * M_1_PI},
    //       {leg_id * 10 + 2, leg->thetalist[1] * 0.5 * M_1_PI},
    //       {leg_id * 10 + 3, leg->thetalist[2] * 0.5 * M_1_PI},
    //   };
    //   if (gotoJointPose(jointAngles) == 1) {
    //     if (leg->state == Leg::LIFT) {
    //       leg->state = Leg::PLACE;
    //     } else if (leg->state == Leg::PLACE) {
    //       leg->state = Leg::RUNNING;
    //     }
    //   }
    // }
  } break;
  }
  if (state != RUNNING && prev_state == RUNNING) {
    for (auto &leg : legs) {
      leg->state = Leg::IDLE;
    }
  }

  teleop->readGamepad();
  queryMotors();
  printStatus();
}

int Robot::homeMotors() {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    cmd_home.velocity = nleg % 2 ? 0.1 : -0.1;
    motorState[nleg][2] =
        motors[nleg][2]->SetPosition(cmd_home, &fmt_home)->values;
  }
  if (prev_state != HOMING)
    return 0;
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    if (motorState[nleg][2].mode == moteus::Mode::kPosition &&
        motorState[nleg][2].fault != 102) {
      return 0;
    }
  }

  // If it has reached its homed position:
  stopMotors();
  ::usleep(1000000); // wait for motors to settle

  // Set home positions for shoulder and upper leg joints
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    for (unsigned njoint; njoint < 3; njoint++) {
      motors[nleg][njoint]->DiagnosticCommand(
          "d cfg-set-output " + std::to_string(home_joint_pose[nleg][njoint]));
    }
  }
  return 1;
}

// int Robot::gotoCartesianPose(const std::map<int, Eigen::Translation3d>
// &legPose,
//                              double max_torque) {
//   std::map<int, double> jointPose;
//   for (auto &pose_pair : legPose) {
//     int leg_id = pose_pair.first;
//     legs[leg_id + 1]->pf = pose_pair.second;
//     legs[leg_id + 1]->ik();
//     // Degrees to turns
//     jointPose[leg_id * 10 + 1] = legs[leg_id + 1]->thetalist[0] * 0.5 *
//     M_1_PI; jointPose[leg_id * 10 + 2] = legs[leg_id + 1]->thetalist[1] *
//     0.5
//     * M_1_PI; jointPose[leg_id * 10 + 3] = legs[leg_id + 1]->thetalist[2] *
//     0.5 * M_1_PI;
//   }

//   // return gotoJointPose(jointPose, max_torque);
//   return 0;
// }

int Robot::gotoJointPose(const JointPose &jointPose, double max_torque) {
  bool all_reached = true;
  const PosFmt pos_fmt{
      .position = moteus::kFloat,
      .velocity = moteus::kFloat,
      .maximum_torque = moteus::kFloat,
      .velocity_limit = moteus::kFloat,
  };
  PosCmd cmd{
      .velocity = NaN,
      .velocity_limit = 1.0,
  };
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    for (unsigned njoint = 0; njoint < 3; njoint++) {
      jointPose[nleg][njoint];
      cmd.position = jointPose[nleg][njoint];
      cmd.maximum_torque = max_torque;
      const auto result = motors[nleg][njoint]->SetPosition(cmd, &pos_fmt);
      if (!result.has_value()) {
        motorState[nleg][njoint] = moteus::Query::Result{};
        continue;
      }
      motorState[nleg][njoint] = result->values;
      if (std::abs(motorState[nleg][njoint].position -
                   jointPose[nleg][njoint]) > 0.01) {
        all_reached = false;
      }
    }
  }
  return (int)all_reached;
}

void Robot::queryMotors() {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    for (unsigned njoint = 0; njoint < 3; njoint++) {
      const auto result = motors[nleg][njoint]->SetQuery();
      if (!result.has_value()) {
        motorState[nleg][njoint] = moteus::Query::Result{};
        continue;
      }
      motorState[nleg][njoint] = result->values;
    }
  }
}

void Robot::stopMotors() {
  for (unsigned nleg = 0; nleg < 4; nleg++) {
    for (unsigned njoint = 0; njoint < 3; njoint++) {
      const auto result = motors[nleg][njoint]->SetStop();
      if (!result.has_value()) {
        motorState[nleg][njoint] = moteus::Query::Result{};
        continue;
      }
      motorState[nleg][njoint] = result->values;
    }
  }
}

void Robot::printStatus() {
  ::printf("\033[2KState: %s\n", state_names[state].c_str());
  if (teleop->error) {
    ::printf("\033[2KGamepad: Not Connected\n");
  } else {
    ::printf("\033[2KGamepad: w=(%6.3f,%6.3f,%6.3f) v=(%6.3f,%6.3f,%6.3f) "
             "home=%1d deploy=%1d\n",
             teleop->V[0], teleop->V[1], teleop->V[2], teleop->V[3],
             teleop->V[4], teleop->V[5], teleop->home_joints,
             teleop->deploy_legs);
  }

  // Print Joint State
  // for (auto &state_pair : motorState) {
  //   int id = state_pair.first;
  //   const auto &r = state_pair.second;
  //   if (isnanl(r->position)) {
  //     ::printf("\033[2K%2d: No data\n", id);
  //     continue;
  //   }
  //   ::printf("\033[2K%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  "
  //            "v/t/f=(%5.1f,%5.1f,%3d)\n",
  //            id, static_cast<int>(r->mode), r->position, r->velocity,
  //            r->torque, r->voltage, r->temperature, r->fault);
  // }
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
