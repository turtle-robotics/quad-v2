#include <iostream>
#include <signal.h>

#include "Robot.hpp"

void signal_callback_handler(int signum)
{
    ::printf("\033[2J\033[H"); // clear screen
    robot.stopMotors();

    exit(signum);
}

// Legs are numbered from 1 to 4: Front Right, Front Left, Back Right, Back Left
// Joints are numbered from 1 to 3: Shoulder, Upper Leg, Lower Leg
// Motor CAN IDs follow: AB where A is the leg number and B is the joint number

int main(int argc, char **argv)
{
    // Handle Ctrl-C signal to stop motors safely
    signal(SIGINT, signal_callback_handler);

    // Load configuration file from command line argument
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }
    std::string config_file = argv[1];

    // Load and parse the YAML configuration file
    std::cout << "Starting robot with config file " << config_file << std::endl;
    YAML::Node config = YAML::LoadFile(config_file);

    if (robot.configure(config) != 0)
    {
        std::cerr << "Failed to configure robot." << std::endl;
        return 1;
    }

    if (robot.init() != 0)
    {
        std::cerr << "Failed to initialize robot." << std::endl;
        return 1;
    }

    std::cout << "Running..." << std::endl;
    while (true)
    {
        robot.loop(10000);
        ::usleep(10000);
    }

    return 0;
}

void Robot::loop(int us)
{
    queryMotors();
    holdPosition();
    printMotorStatus();
}

using Transport = pi3hat::Pi3HatMoteusTransport;

// variables prefixed with c_ are YAML nodes
int Robot::configure(YAML::Node conf)
{
    if (configured)
    {
        std::cerr << "Robot is already configured." << std::endl;
        return -1;
    }

    std::cout << "Configuring robot..." << std::endl;

    // Create mjbots pi3hat transport
    Transport::Options toptions;

    const auto &c_servomap = conf["servomap"];
    for (auto &c_bus_id : c_servomap)
    {
        int bus = c_bus_id.first.as<int>();
        for (auto &c_servo_id : c_bus_id.second)
        {
            toptions.servo_map[c_servo_id.as<int>()] = bus;
        }
        ::fflush(stdout);
    }
    const auto transport = std::make_shared<Transport>(toptions);

    // Create motors
    for (int leg_id = 1; leg_id <= 4; leg_id++)
    {
        for (int joint_id = 1; joint_id <= 3; joint_id++)
        {
            int can_id = leg_id * 10 + joint_id;

            motors[can_id] = new moteus::Controller([&]()
                                                    {
    moteus::Controller::Options coptions;
    coptions.id=can_id;
    coptions.transport = transport;
    return coptions; }());
        }
    }

    // Create legs
    const auto &c_leg = conf["leg"];
    float lengths[3] = {
        c_leg["upperLen"].as<float>(),
        c_leg["lowerLen"].as<float>(),
        c_leg["shoulderLen"].as<float>()};
    float footRadius = c_leg["footRadius"].as<float>();
    // for (int i = 1; i <= 4; i++)
    //     legs[i] = new Leg(lengths, footRadius);

    // Create chassis
    const auto &c_chassis = conf["chassis"];
    if (c_chassis["width"].IsNull() || c_chassis["length"].IsNull())
    {
        std::cerr << "Invalid configuration file: missing chassis parameters." << std::endl;

        return -1;
    }
    // Chassis chassis_inst{
    //     c_chassis["width"].as<float>(),
    //     c_chassis["length"].as<float>(),
    //     LF, RF, LB, RB};

    std::cout << "Robot configured successfully." << std::endl;
    configured = true;
    return 0;
}

int Robot::init()
{
    if (!configured)
    {
        std::cerr << "Robot is not configured." << std::endl;
        return -1;
    }
    if (initialized)
    {
        std::cerr << "Robot is already initialized." << std::endl;
        return -1;
    }

    std::cout << "Initializing robot..." << std::endl;

    // Setup each motor
    for (auto &motor_pair : motors)
    {
        int id = motor_pair.first;
        moteus::Controller *motor = motor_pair.second;
        // set servo.min_position and servo.max_position
        const auto state = motor->SetStop();
        motorState[id] = state->values;
        moteus::GpioWrite::Command gpio_cmd;
        gpio_cmd.aux1 = 1; // set aux1 to high
        motor->SetWriteGpio(gpio_cmd);
    }

    // Initialize motors
    stopMotors();

    ::usleep(10000); // small delay

    std::cout << "Robot initialized successfully." << std::endl;
    initialized = true;
    return 0;
}

void Robot::queryMotors()
{
    for (auto &motor_pair : motors)
    {
        int id = motor_pair.first;
        moteus::Controller *motor = motor_pair.second;

        auto result = motor->SetQuery();
        if (!result.has_value())
        {
            motorState[id] = moteus::Query::Result{};
            continue;
        }
        motorState[id] = result->values;
    }
}

void Robot::stopMotors()
{
    for (auto &motor_pair : motors)
    {
        int id = motor_pair.first;
        moteus::Controller *motor = motor_pair.second;

        const auto state = motor->SetStop();
        motorState[id] = state->values;
    }
}

void Robot::holdPosition()
{
    for (auto &motor_pair : motors)
    {
        int id = motor_pair.first;
        moteus::Controller *motor = motor_pair.second;

        const auto state = motor->SetZeroVelocity();
        motorState[id] = state->values;
    }
}

void Robot::printMotorStatus()
{
    for (auto &state_pair : motorState)
    {
        int id = state_pair.first;
        const auto &r = state_pair.second;
        if (isnanl(r.position))
        {
            ::printf("%2d:  No data                                                 \n", id);
            continue;
        }
        ::printf("%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \n",
                 id,
                 static_cast<int>(r.mode),
                 r.position,
                 r.velocity,
                 r.torque,
                 r.voltage,
                 r.temperature,
                 r.fault);
    }
    ::printf("\033[%dA", motorState.size());
    ::fflush(stdout);
}
