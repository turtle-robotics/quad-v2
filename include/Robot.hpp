#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <moteus.h>
#include <string>
#include <yaml-cpp/yaml.h>
#if defined(__aarch64__)
#include <pi3hat_moteus_transport.h>
#endif

#include "Chassis.hpp"
#include "Leg.hpp"
#include "Teleop.hpp"
#include "helper.hpp"

using namespace mjbots;
class Robot {
  using PosFmt = moteus::PositionMode::Format;
  using MotorState = moteus::Query::Result;
  using Controller = moteus::Controller;
  using Resolution = moteus::Resolution;

public:
  typedef LEG_JOINT_ARRAY(std::shared_ptr<moteus::Controller>) Motors;
  typedef LEG_JOINT_ARRAY(double) JointPose;

  Robot(std::shared_ptr<Chassis> chassis, LEG_ARRAY(std::shared_ptr<Leg>) legs,
        std::shared_ptr<Teleop> teleop, Motors motors)
      : chassis{chassis}, legs{legs}, teleop{teleop}, motors{motors} {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop(unsigned int us);

  void stopMotors();
  void queryMotors();
  void printStatus();
  int gotoJointPose(const JointPose &jointPose, double max_torque = NaN);

  int homeMotors();

  std::string status;

private:
  bool configured = false;
  bool initialized = false;
  bool legs_deployed = false;
  bool gamepad_error = false;
  // bool enabled = false;
  enum state_t {
    IDLE,
    HOMING,
    DEPLOY_A,
    DEPLOY_B,
    DEPLOY_C,
    RUNNING
  } state = IDLE,
    prev_state = IDLE;
  std::map<state_t, std::string> state_names{
      {IDLE, "IDLE"},         {HOMING, "HOMING"},     {DEPLOY_A, "DEPLOY_A"},
      {DEPLOY_B, "DEPLOY_B"}, {DEPLOY_C, "DEPLOY_C"}, {RUNNING, "RUNNING"},
  };
  int last_leg_id = 0;

  const std::shared_ptr<Chassis> chassis;
  const LEG_ARRAY(std::shared_ptr<Leg>) legs;
  const std::shared_ptr<Teleop> teleop;
  const Motors motors;

  LEG_JOINT_ARRAY(moteus::Query::Result) motorState;
  LEG_JOINT_ARRAY(PosCmd) motorPosCmds;

#if defined(__aarch64__)
  pi3hat::Attitude attitude;
#endif
  double lower_min, lower_max;

  // Homing parameters
  double max_homing_torque = 3.5;
  PosFmt fmt_home{
      .position = Resolution::kIgnore,
      .velocity = Resolution::kFloat,
      .maximum_torque = Resolution::kFloat,
      .ignore_position_bounds = Resolution::kFloat,
  };
  PosCmd cmd_home{
      .position = NaN,
      .maximum_torque = max_homing_torque,
      .ignore_position_bounds = 1.0,
  };
  PosFmt fmt_deploy{
      .position = Resolution::kFloat,
      .velocity = Resolution::kIgnore,
      .maximum_torque = Resolution::kFloat,
      .velocity_limit = Resolution::kFloat,
  };
  PosCmd cmd_deploy{
      .maximum_torque = 2.5,
      .velocity_limit = 0.5,
  };

  double deploy_torque = 2.5; // N m
  double deploy_vel = 0.1;    // m/s

  // Predefined joint poses
  JointPose home_joint_pose;
  JointPose deploy_a_cmds;
  JointPose deploy_b_cmds;
  JointPose deploy_c_cmds;
};