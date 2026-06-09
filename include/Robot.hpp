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
  Robot(Chassis chassis, LegArray<std::shared_ptr<Leg>> legs, HID hid,
        Motors motors, std::map<uint32_t, unsigned> can_map,
        std::shared_ptr<pi3hat::Pi3HatMoteusTransport> transport)
      : chassis{chassis}, legs{legs}, hid{hid}, motors{motors},
        can_map{can_map}, transport{transport} {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop();

  void stopMotors();
  void queryMotors();
  void printStatus();
  bool gotoJointPose(const JointPose &jointPose, double vel_lim = NaN,
                     double max_torque = NaN);
  void run();
  bool homeMotors();

  std::string status;
  Eigen::Quaterniond attitude;

  enum State {
    IDLE,
    HOMING,
    DEPLOY_A,
    DEPLOY_B,
    DEPLOY_C,
    RUNNING,
    EXITING
  } state = IDLE,
    prev_state = IDLE;

private:
  bool configured = false;
  bool initialized = false;
  bool legs_deployed = false;
  bool gamepad_error = false;
  // bool enabled = false;

  std::map<State, std::string> state_names{
      {IDLE, "IDLE"},         {HOMING, "HOMING"},     {DEPLOY_A, "DEPLOY_A"},
      {DEPLOY_B, "DEPLOY_B"}, {DEPLOY_C, "DEPLOY_C"}, {RUNNING, "RUNNING"},
      {EXITING, "EXITING"},
  };

  struct HomingState {
    enum { STARTUP, MOVING, WAITING, HOMED } state = STARTUP;
    size_t cycle_count = 0;
    void reset() { *this = {}; }
  };

  LegArray<HomingState> homing_states;

  int last_leg_id = 0;

  Chassis chassis;
  LegArray<std::shared_ptr<Leg>> legs;
  HID hid;
  Motors motors;

  std::map<uint32_t, unsigned> can_map;

  LegJointArray<moteus::Query::Result> motorState;
  LegJointArray<PosCmd> motorPosCmds;

  pi3hat::Attitude imu;
  std::shared_ptr<pi3hat::Pi3HatMoteusTransport> transport;
  // Handling CAN FD frames
  LegJointArray<moteus::CanFdFrame> frames;
  std::vector<moteus::CanFdFrame> replies;

  // Homing parameters
  const PosFmt fmt_home{
      .position = Resolution::kFloat,
      .velocity = Resolution::kFloat,
      .feedforward_torque = Resolution::kFloat,
      .maximum_torque = Resolution::kFloat,
      .ignore_position_bounds = Resolution::kFloat,
  };
  const PosFmt fmt_run{
      .position = Resolution::kFloat,
      .velocity = Resolution::kIgnore,
      .feedforward_torque = Resolution::kIgnore,
      .maximum_torque = Resolution::kFloat,
      .velocity_limit = Resolution::kFloat,
  };
  double deploy_torque = 2.5; /// N m
  double deploy_vel = 1.0;    /// m/s

  // Predefined joint poses
  JointPose home_joint_pose;
  JointPose deploy_a_cmds;
  JointPose deploy_b_cmds;
  JointPose deploy_c_cmds;

  bool setJointPos(const JointPose &jointPos);
  int updateMotorState();
  void cycleFrames();
};