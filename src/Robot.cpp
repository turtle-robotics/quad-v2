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
  std::cout << "Configuring real-time" << std::endl;
  mjbots::pi3hat::ConfigureRealtime(0);

  // Create motors
  home_joint_pose = conf["home_pose"].as<JointPose>();
  deploy_a_cmds = conf["deploy_cmds"]["a"].as<JointPose>();
  deploy_b_cmds = conf["deploy_cmds"]["b"].as<JointPose>();
  deploy_c_cmds = conf["deploy_cmds"]["c"].as<JointPose>();

  if (configure_motors) {
    std::cout << "Writing motor configuration..." << std::endl;

    std::string conf_str;
    for (const auto &config_line : conf["motors"]["config"])
      for (const auto &motor : motors) {
        conf_str = "conf set " + config_line.as<std::string>();
        motor->DiagnosticCommand(conf_str);
        // TODO: Add to logging functionality
        // ::printf("Wrote config \"%s\" to motor %d\n", conf_str.c_str(),
        //          motor_pair.first);
      }
  }
  if (write_motor_config) {
    std::cout << "Writing motor configuration to flash..." << std::endl;
    for (const auto &motor : motors) {
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
  if (hid.init() != 0) {
    std::cerr << "Failed to initialize teleop." << std::endl;
    return -1;
  }

  ::usleep(10000); // small delay

  std::cout << "Robot initialized successfully." << std::endl;
  initialized = true;
  return 0;
}

void Robot::loop() {
  prev_state = state;

  // Set all frames to query, to be overwritten later
  queryMotors();

  // Change State
  switch (state) {
  case IDLE: {
    if (hid.home_joints)
      state = HOMING;
    else if (hid.deploy_legs)
      if (legs_deployed)
        state = DEPLOY_C;
      else
        state = DEPLOY_A;
  } break;
  case HOMING: {
    if (!hid.home_joints)
      state = IDLE;
  } break;
  case DEPLOY_A:
  case DEPLOY_B:
  case DEPLOY_C: {
    if (!hid.deploy_legs)
      state = IDLE;
  } break;
  case RUNNING: {
    // Homing not allowed while running
    if (hid.home_joints)
      hid.home_joints = false;

    if (hid.deploy_legs)
      if (legs_deployed)
        state = DEPLOY_C;
      else
        state = DEPLOY_A;
  } break;
  }
  // Act on State
  switch (state) {
  case IDLE: {
    stopMotors();
  } break;
  case HOMING: {
    if (homeMotors()) {
      hid.home_joints = false;
      state = IDLE;
    }
  } break;
  case DEPLOY_A: {
    if (gotoJointPose(deploy_a_cmds, deploy_vel, deploy_torque)) {
      if (legs_deployed) {
        hid.deploy_legs = false;
        legs_deployed = false;
        state = IDLE;
      } else {
        state = DEPLOY_B;
      }
    }
  } break;
  case DEPLOY_B: {
    if (gotoJointPose(deploy_b_cmds, deploy_vel, deploy_torque)) {
      if (legs_deployed) {
        state = DEPLOY_A;
      } else {
        state = DEPLOY_C;
      }
    }
  } break;
  case DEPLOY_C: {
    if (gotoJointPose(deploy_c_cmds, deploy_vel, deploy_torque)) {
      if (legs_deployed) {
        state = DEPLOY_B;
      } else {
        state = RUNNING;
        hid.deploy_legs = false;
        legs_deployed = true;
      }
    }
  } break;
  case RUNNING: {
    // TODO: Gotta figure this one out

    // chassis.Vb = hid.V;
    // chassis.run();
    // for (unsigned i = 0; i < 4; i++) {
    //   legs[i]->run();
    //   auto poscmd =
    //       makePosCmd(legs[i]->thetalist, legs[i]->thetadlist,
    //       legs[i]->taulist);

    //   motorPosCmds[i * 3 + 0] = poscmd[0];
    //   motorPosCmds[i * 3 + 1] = poscmd[1];
    //   motorPosCmds[i * 3 + 2] = poscmd[2];
    // }

    // int leg_id = 0;
    // for (auto &leg : legs) {
    //   leg_id++;
    //   Eigen::Vector3d v = 0.2 * teleop.V.tail<3>(); // body velocity in x, y,
    //   z
    //   // leg->walk(v);
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
  case EXITING: {
    stopMotors();
  } break;
  }
  if (state != RUNNING && prev_state == RUNNING) {
    for (auto &leg : legs) {
      leg->state = Leg::IDLE;
    }
  }

  hid.readGamepad();
  cycleFrames();
  printStatus();
}

bool Robot::homeMotors() {
  static const double homing_max_torque = 2.5;
  static const double homing_vel = 0.4;

  if (prev_state != HOMING) {
    std::cout << "Homing" << std::endl;
  }

  unsigned i = 2;
  bool all_homed = true;

  for (HomingState &state : homing_states) {
    if (prev_state != HOMING) {
      state.reset();
    }
    if (state.state < HomingState::WAITING) {
      int fwd = (i / 3) % 2 ? 1 : -1;
      frames[i] = motors[i]->MakePosition(
          PosCmd{
              .position = NaN,
              .velocity = fwd * homing_vel,
              .maximum_torque = homing_max_torque,
              .ignore_position_bounds = 1.0,
          },
          &fmt_home);
    }
    switch (state.state) {
    case HomingState::STARTUP: {
      if (motorState[i].mode == moteus::Mode::kPosition)
        state.state = HomingState::MOVING;
      all_homed = false;
    } break;
    case HomingState::MOVING: {
      if (motorState[i].mode != moteus::Mode::kPosition ||
          motorState[i].fault == 102)
        state.state = HomingState::WAITING;
      all_homed = false;
    } break;
    case HomingState::WAITING: {
      state.cycle_count++;
      if (state.cycle_count > 250)
        state.state = HomingState::HOMED;
      all_homed = false;
    } break;
    case HomingState::HOMED: {
    } break;
    }
    i += 3;
  }

  if (all_homed)
    setJointPos(home_joint_pose);

  return all_homed;
}

void Robot::run() {
  for (auto &leg : legs) {
    leg->pf = Eigen::Vector3d{};
    leg->run();
  }
}

// int Robot::gotoCartesianPose(const std::map<int, Eigen::Vector3d>
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

bool Robot::gotoJointPose(const JointPose &jointPose, double vel_lim,
                          double max_torque) {
  bool all_reached = true;
  for (size_t i = 0; i < 12; i++) {
    frames[i] = motors[i]->MakePosition(
        PosCmd{
            .position = jointPose(i),
            // .velocity = NaN,
            // .feedforward_torque = NaN,
            .maximum_torque = max_torque,
            .velocity_limit = vel_lim,
        },
        &fmt_run);
    if (std::abs(motorState[i].position - jointPose(i)) > 0.01) {
      all_reached = false;
    }
  }
  return all_reached;
}

void Robot::queryMotors() {
  for (unsigned i = 0; i < 12; i++) {
    frames[i] = motors[i]->MakeQuery();
  }
}

void Robot::stopMotors() {
  for (unsigned i = 0; i < 12; i++) {
    frames[i] = motors[i]->MakeStop();
  }
}

void Robot::printStatus() {
  ::printf("\033[2KState: %s\n", state_names[state].c_str());
  if (hid.error) {
    ::printf("\033[2KGamepad: Not Connected\n");
  } else {
    ::printf("\033[2KGamepad: w=(%6.3f,%6.3f,%6.3f) v=(%6.3f,%6.3f,%6.3f) "
             "home=%1d deploy=%1d\n",
             hid.V[0], hid.V[1], hid.V[2], hid.V[3], hid.V[4], hid.V[5],
             hid.home_joints, hid.deploy_legs);
  }

  // for (size_t i = 0; i < 12; i++) {
  //   std::cout << "mode: " << static_cast<int>(motorState[i].mode) << ", "
  //             << "position: " << motorState[i].position << ", "
  //             << "velocity: " << motorState[i].velocity << ", "
  //             << "torque: " << motorState[i].torque << ", "
  //             << "q_current: " << motorState[i].q_current << ", "
  //             << "d_current: " << motorState[i].d_current << ", "
  //             << "abs_position: " << motorState[i].abs_position << ", "
  //             << "power: " << motorState[i].power << ", "
  //             << "motor_temperature: " << motorState[i].motor_temperature
  //             << ", "
  //             << "trajectory_complete: " << motorState[i].trajectory_complete
  //             << ", "
  //             << "home_state: " << static_cast<int>(motorState[i].home_state)
  //             << ", "
  //             << "voltage: " << motorState[i].voltage << ", "
  //             << "temperature: " << motorState[i].temperature << ", "
  //             << "fault: " << motorState[i].fault << ", "
  //             << "aux1_gpio: " << motorState[i].aux1_gpio << ", "
  //             << "aux2_gpio: " << motorState[i].aux2_gpio << std::endl;
  // }

  // Print Joint State
  size_t i = 0;
  for (auto &r : motorState) {
    if (isnanl(r.position)) {
      ::printf("\033[2K%2d: No data\n", can_map.find(i)->second);
      continue;
    }
    ::printf("\033[2K%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  "
             "v/t/f=(%5.1f,%5.1f,%3d)\n",
             can_map.find(i)->second, static_cast<int>(r.mode), r.position,
             r.velocity, r.torque, r.voltage, r.temperature, r.fault);
    i++;
  }
  // int leg_id = 0;
  // for (auto &leg : legs) {
  //   leg_id++;
  //   ::printf("\033[2KLeg %d: state=%s p=(%6.3f,%6.3f,%6.3f) "
  //            "theta=(%6.3f,%6.3f,%6.3f)\n",
  //            leg_id,
  //            leg->state == Leg::IDLE      ? "IDLE   "
  //            : leg->state == Leg::LIFT    ? "LIFT   "
  //            : leg->state == Leg::PLACE   ? "PLACE  "
  //            : leg->state == Leg::RUNNING ? "RUNNING"
  //                                         : "UNKNOWN",
  //            leg->pf.x(), leg->pf.y(), leg->pf.z(), leg->thetalist[0],
  //            leg->thetalist[1], leg->thetalist[2]);
  // }
  // ::printf("\033[%dA", motorState.size() + 2);
  ::fflush(stdout);
}

bool Robot::setJointPos(const JointPose &jointPos) {
  for (size_t i = 0; i < 12; i++) {
    motors[i]->DiagnosticCommand("d cfg-set-output " +
                                 std::to_string(jointPos(i)));
  }
  return true;
}

void Robot::cycleFrames() {
  for (size_t i = 0; i < frames.size(); i += 4) {
    moteus::BlockingCallback cbk;
    transport->Cycle(frames.data() + i, 4UL, &replies, &imu, nullptr, nullptr,
                     cbk.callback());
    cbk.Wait();
    for (const auto &reply : replies) {
      motorState[can_map[reply.source]] =
          mjbots::moteus::Query::Parse(reply.data, reply.size);
    }
    replies.clear();
  }

  attitude.w() = imu.attitude.w;
  attitude.x() = imu.attitude.x;
  attitude.y() = imu.attitude.y;
  attitude.z() = imu.attitude.z;
  chassis.Ts0.linear() = attitude.toRotationMatrix();
  // std::cout << attitude << std::endl;

  // for (unsigned i = 0; i < 12; i++) {
  //   motors[i]->options().can_prefix;
  //   motors[i]->options().id;
  //   motors[i]->options().bus;
  // }

  // chassis.Vdb(3) = imu.accel_mps2.x;
  // chassis.Vdb(4) = imu.accel_mps2.y;
  // chassis.Vdb(5) = imu.accel_mps2.z;
  // chassis.Vb(0) = imu.rate_dps.x;
  // chassis.Vb(1) = imu.rate_dps.y;
  // chassis.Vb(2) = imu.rate_dps.z;
}
