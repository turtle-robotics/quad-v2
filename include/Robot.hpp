#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <moteus.h>
#include <pi3hat_moteus_transport.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include "Chassis.hpp"
#include "HID.hpp"
#include "Leg.hpp"
#include "helper.hpp"

using namespace mjbots;
class Robot {
  using PosFmt = moteus::PositionMode::Format;
  using MotorState = moteus::Query::Result;
  using Controller = moteus::Controller;
  using Resolution = moteus::Resolution;

public:
  Robot(Chassis chassis, LegArray<Leg> legs, HID hid, Motors motors,
        std::shared_ptr<pi3hat::Pi3HatMoteusTransport> transport)
      : chassis{chassis}, legs{legs}, hid{hid}, motors{motors},
        transport{transport} {};

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
  Eigen::Quaterniond attitude;

private:
  bool configured = false;
  bool initialized = false;
  bool legs_deployed = false;
  bool gamepad_error = false;
  // bool enabled = false;
  enum State {
    IDLE,
    HOMING,
    DEPLOY_A,
    DEPLOY_B,
    DEPLOY_C,
    RUNNING
  } state = IDLE,
    prev_state = IDLE;
  std::map<State, std::string> state_names{
      {IDLE, "IDLE"},         {HOMING, "HOMING"},     {DEPLOY_A, "DEPLOY_A"},
      {DEPLOY_B, "DEPLOY_B"}, {DEPLOY_C, "DEPLOY_C"}, {RUNNING, "RUNNING"},
  };
  int last_leg_id = 0;

  Chassis chassis;
  LegArray<Leg> legs;
  HID hid;
  Motors motors;

  LegJointArray<moteus::Query::Result> motorState;
  LegJointArray<PosCmd> motorPosCmds;
  
  pi3hat::Attitude imu;
  std::shared_ptr<pi3hat::Pi3HatMoteusTransport> transport;
  // Handling CAN FD frames
  std::vector<moteus::CanFdFrame> frames;
  std::vector<moteus::CanFdFrame> replies;

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

  bool setJointPos(JointPose &jointPos);
  void cycleFrames();
};