#pragma once
#include "Chassis.hpp"
#include <yaml-cpp/yaml.h>
#include <map>
#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace mjbots;
class Robot
{
public:
    Robot() {};
    // Robot(Chassis chassis) : chassis{chassis} {};

    // Configure robot using a YAML configuration file
    int configure(YAML::Node conf);
    int init();
    void loop(int us);

    void stopMotors();
    void queryMotors();
    void holdPosition();
    void printMotorStatus();

private:
    bool configured = false;
    bool initialized = false;
    std::map<int, Leg *> legs;
    std::map<int, moteus::Controller *> motors;
    std::map<int, moteus::Query::Result> motorState;
    Chassis *chassis;
} robot;