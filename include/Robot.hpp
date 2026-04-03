#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
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

using namespace mjbots;
class Robot {
  using PosCmd = moteus::PositionMode::Command;
  using PosFmt = moteus::PositionMode::Format;
  using MotorState = moteus::Query::Result;
  using Controller = moteus::Controller;
  using Resolution = moteus::Resolution;

public:
  typedef std::array<std::array<std::shared_ptr<moteus::Controller>, 3>, 4>
      Motors;
  typedef std::array<std::array<double, 3>, 4> JointPose;

  Robot(std::shared_ptr<Chassis> chassis,
        std::array<std::shared_ptr<Leg>, 4> legs,
        std::shared_ptr<Teleop> teleop, Motors motors)
      : chassis{chassis}, legs{legs}, teleop{teleop}, motors{motors} {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop(unsigned int us);

  void stopMotors();
  void queryMotors();
  // void holdPosition();
  void printStatus();
  // int gotoCartesianPose(const std::map<int, Eigen::Translation3d> &legPose,
  //                       double max_torque = NaN);
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

  double chassis_length, chassis_width;

  const std::shared_ptr<Chassis> chassis;
  const std::array<std::shared_ptr<Leg>, 4> legs;
  const std::shared_ptr<Teleop> teleop;
  const Motors motors;

  std::array<std::array<moteus::Query::Result, 3>, 4> motorState;

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
      .maximum_torque = max_homing_torque,
      .ignore_position_bounds = 1.0,
  };

  double deploy_torque = 2.5; // N m
  double deploy_vel = 0.1;    // m/s

  // Predefined joint poses
  JointPose home_joint_pose;
  JointPose deploy_a_cmds;
  JointPose deploy_b_cmds;
  JointPose deploy_c_cmds;

  // const std::map<int, double> deploy_a_cmds{
  //     {11, +0.250}, {12, -0.000}, {13, +0.100}, //
  //     {21, -0.250}, {22, -0.000}, {23, +0.100}, //
  //     {31, +0.250}, {32, -0.000}, {33, +0.100}, //
  //     {41, -0.250}, {42, -0.000}, {43, +0.100}, //
  // };
  // const std::map<int, double> deploy_b_cmds{
  //     {11, +0.125}, {12, -0.000}, {13, +0.100}, //
  //     {21, -0.125}, {22, -0.000}, {23, +0.100}, //
  //     {31, +0.125}, {32, -0.000}, {33, +0.100}, //
  //     {41, -0.125}, {42, -0.000}, {43, +0.100}, //
  // };
  // const std::map<int, double> deploy_c_cmds{
  //     {11, +0.000}, {12, -0.125}, {13, +0.100}, //
  //     {21, -0.000}, {22, -0.125}, {23, +0.100}, //
  //     {31, +0.000}, {32, -0.125}, {33, +0.100}, //
  //     {41, -0.000}, {42, -0.125}, {43, +0.100}, //
  // };
};