/*
 * MIT License
 *
 * Copyright (c) 2026 Ian Lansdowne
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include "Robot.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

Robot robot;

// Legs are numbered from 1 to 4: Front Right, Front Left, Back Right, Back Left
// Joints are numbered from 1 to 3: Shoulder, Upper Leg, Lower Leg
// Motor CAN IDs follow: AB where A is the leg number and B is the joint number

int main(int argc, char **argv)
{

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

    if (config.IsNull() || config["name"].IsNull())
    {
        std::cerr << "Failed to load configuration file: " << config_file << std::endl;
        return 1;
    }

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
        usleep(10000);
    }

    return 0;
}