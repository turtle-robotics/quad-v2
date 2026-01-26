#pragma once
#include <moteus.h>
#include <pi3hat_moteus_transport.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <map>

#include "Chassis.hpp"
#include "Teleop.hpp"

using namespace mjbots;
class Robot {
public:
  Robot() {};
  // Robot(Chassis chassis) : chassis{chassis} {};

  // Configure robot using a YAML configuration file
  int configure(YAML::Node conf, bool configure_motors,
                bool write_motor_config);
  int init();
  void loop(int us);

  void stopMotors();
  void queryMotors();
  void holdPosition();
  void gotoZero();
  void printMotorStatus();

  void calibrate();

private:
  bool configured = false;
  bool initialized = false;
  bool enabled = false;
  enum state_t {
    IDLE,
    INITIALIZING,
    CONFIGURING,
    HOMING,
    RUNNING
  } state = IDLE;
  std::map<int, Leg *> legs;
  std::map<int, moteus::Controller *> motors;
  std::map<int, moteus::Query::Result> motorState;
  Chassis *chassis;
  Teleop teleop;
} robot;