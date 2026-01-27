#pragma once
#include <Eigen/Core>
#include <iostream>
#include <map>
#include <moteus.h>
#include <pi3hat_moteus_transport.h>
#include <yaml-cpp/yaml.h>

#include "Chassis.hpp"
#include "Teleop.hpp"

using namespace mjbots;
using PosCmd = moteus::PositionMode::Command;
using PosFmt = moteus::PositionMode::Format;

class Robot {
public:
  Robot() {};
  // Robot(Chassis chassis) : chassis{chassis} {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop(unsigned int us);

  void stopMotors();
  void queryMotors();
  void holdPosition();
  void gotoZero();
  void printMotorStatus();
  int gotoPose(std::map<int, double> jointPose);

  int homeMotors();

  std::string status;

private:
  bool configured = false;
  bool initialized = false;
  bool legs_deployed = false;
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
  std::map<int, Leg *> legs;
  std::map<int, moteus::Controller *> motors;
  std::map<int, moteus::Query::Result> motorState;
  Chassis *chassis;
  Teleop teleop;

  double lower_min, lower_max;

  // Homing parameters
  double max_homing_torque = 2.5;
  PosFmt homing_pos_fmt{
      .position = moteus::kFloat,
      .velocity = moteus::kFloat,
      .maximum_torque = moteus::kFloat,
      .ignore_position_bounds = moteus::kFloat,
  };
  PosCmd homing_left_cmd{
      .position = NaN,
      .velocity = -0.1,
      .maximum_torque = max_homing_torque,
      .ignore_position_bounds = 1.0,
  };
  PosCmd homing_right_cmd{
      .position = NaN,
      .velocity = 0.1,
      .maximum_torque = max_homing_torque,
      .ignore_position_bounds = 1.0,
  };

  // Deployment parameters
  std::map<int, double> deploy_a_cmds{
      // {11, -0.25}, {12, 0.0}, {13, -0.125},
      // {21, 0.25},
      // {22, 0.0},
      // {23, 0.1},
      // {31, -0.25}, {32, 0.0}, {33, -0.125},
      {41, -0.25},
      {42, 0.0},
      {43, 0.1},
  };
  std::map<int, double> deploy_b_cmds{
      // {11, -0.25}, {12, 0.0}, {13, -0.125},
      // {21, 0.125},
      // {22, 0.125},
      // {23, 0.1},
      // {31, -0.25}, {32, 0.0}, {33, -0.125},
      {41, -0.125},
      {42, 0.125},
      {43, 0.1},
  };
  std::map<int, double> deploy_c_cmds{
      // {11, -0.25}, {12, 0.0}, {13, -0.125},
      // {21, 0.0},
      // {22, 0.125},
      // {23, 0.1},
      // {31, -0.25}, {32, 0.0}, {33, -0.125},
      {41, 0.0},
      {42, 0.125},
      {43, 0.1},
  };
};