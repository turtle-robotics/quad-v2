#pragma once
#include <Eigen/Core>
#include <iostream>
#include <map>
#include <moteus.h>
#include <pi3hat_moteus_transport.h>
#include <yaml-cpp/yaml.h>

#include "Leg.hpp"
#include "Teleop.hpp"

using namespace mjbots;
using PosCmd = moteus::PositionMode::Command;
using PosFmt = moteus::PositionMode::Format;

class Robot {
public:

  Robot() {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop(unsigned int us);

  void stopMotors();
  void queryMotors();
  void holdPosition();
  void printStatus();
  int gotoPose(std::map<int, double> jointPose, double max_torque = NaN);

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
  std::map<state_t, std::string> state_names{
      {IDLE, "IDLE"},         {HOMING, "HOMING"},     {DEPLOY_A, "DEPLOY_A"},
      {DEPLOY_B, "DEPLOY_B"}, {DEPLOY_C, "DEPLOY_C"}, {RUNNING, "RUNNING"},
  };

  double chassis_length, chassis_width;

  std::map<int, Leg *> legs;
  std::map<int, moteus::Controller *> motors;
  std::map<int, moteus::Query::Result> motorState;
  Teleop teleop;

  double lower_min, lower_max;

  // Homing parameters
  double max_homing_torque = 3.5;
  PosFmt homing_pos_fmt{
      .position = moteus::kFloat,
      .velocity = moteus::kFloat,
      .maximum_torque = moteus::kFloat,
      .ignore_position_bounds = moteus::kFloat,
  };
  PosCmd homing_cmd{
      .position = NaN,
      .velocity = -0.1,
      .maximum_torque = max_homing_torque,
      .ignore_position_bounds = 1.0,
  };

  char lower_str[64];

  double deploy_torque = 2.5; // N m
  // Deployment parameters
  std::map<int, double> deploy_a_cmds{
      {11, 0.250}, {12, 0.000}, {13, 0.100}, //
      {21, 0.250}, {22, 0.000}, {23, 0.100}, //
      {31, 0.250}, {32, 0.000}, {33, 0.100}, //
      {41, 0.250}, {42, 0.000}, {43, 0.100}, //
  };
  std::map<int, double> deploy_b_cmds{
      {11, 0.125}, {12, 0.125}, {13, 0.100}, //
      {21, 0.125}, {22, 0.125}, {23, 0.100}, //
      {31, 0.125}, {32, 0.125}, {33, 0.100}, //
      {41, 0.125}, {42, 0.125}, {43, 0.100}, //
  };
  std::map<int, double> deploy_c_cmds{
      {11, 0.000}, {12, 0.125}, {13, 0.100}, //
      {21, 0.000}, {22, 0.125}, {23, 0.100}, //
      {31, 0.000}, {32, 0.125}, {33, 0.100}, //
      {41, 0.000}, {42, 0.125}, {43, 0.100}, //
  };

  std::map<int, double> stand_cmds{
      {11, 0.000}, {12, -0.110}, {13, 0.250}, //
      {21, 0.000}, {22, -0.110}, {23, 0.250}, //
      {31, 0.000}, {32, -0.090}, {33, 0.250}, //
      {41, 0.000}, {42, -0.090}, {43, 0.250}, //
  };

  std::map<int, Eigen::Vector3d> foot_positions{
      {1, {0.000, 0.010, 0.200}}, //
      {2, {0.000, 0.010, 0.200}}, //
      {3, {0.000, 0.010, 0.200}}, //
      {4, {0.000, 0.010, 0.200}}, //
  };
};