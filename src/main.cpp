#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include "Robot.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

Robot *robot;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }
    std::string config_file = argv[1];
    std::cout << "Starting robot with config file " << config_file << std::endl;
    YAML::Node config = YAML::LoadFile(config_file);
    if (config.IsNull() || config["name"].IsNull())
    {
        std::cerr << "Failed to load configuration file: " << config_file << std::endl;
        return 1;
    }
    std::cout << "Config file for " << config["name"] << " loaded successfully." << std::endl;
    std::cout << "Initializing robot..." << std::endl;

    if (config["chassis"].IsNull() || config["legs"].IsNull() || config["leg"].IsNull())
    {
        std::cerr << "Invalid configuration file: missing 'legs' or 'chassis' section." << std::endl;
        return 1;
    }

    const auto &leg = config["leg"];

    if (leg["shoulderLen"].IsNull() || leg["upperLen"].IsNull() || leg["lowerLen"].IsNull() || leg["footRadius"].IsNull())
    {
        std::cerr << "Invalid configuration file: missing leg parameters." << std::endl;
        return 1;
    }
    Leg LF{
        leg["upperLen"].as<float>(),
        leg["lowerLen"].as<float>(),
        leg["footRadius"].as<float>(),
        leg["shoulderLen"].as<float>()};
    Leg RF{
        leg["upperLen"].as<float>(),
        leg["lowerLen"].as<float>(),
        leg["footRadius"].as<float>(),
        -leg["shoulderLen"].as<float>()};
    Leg LB{
        leg["upperLen"].as<float>(),
        leg["lowerLen"].as<float>(),
        leg["footRadius"].as<float>(),
        leg["shoulderLen"].as<float>()};
    Leg RB{
        leg["upperLen"].as<float>(),
        leg["lowerLen"].as<float>(),
        leg["footRadius"].as<float>(),
        -leg["shoulderLen"].as<float>()};
    const auto &chassis = config["chassis"];
    if (chassis["width"].IsNull() || chassis["length"].IsNull())
    {
        std::cerr << "Invalid configuration file: missing chassis parameters." << std::endl;
        return 1;
    }
    Chassis chassis_inst{
        chassis["width"].as<float>(),
        chassis["length"].as<float>(),
        LF, RF, LB, RB};

    Robot robot{chassis_inst};
    
    std::cout << "Robot initialized successfully." << std::endl;

    std::cout << "Running..." << std::endl;
    while (true)
    {
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}