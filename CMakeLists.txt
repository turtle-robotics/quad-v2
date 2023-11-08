cmake_minimum_required(VERSION 3.8)
project(
  TurtleQUAD
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_CXX_FLAGS -static-libstdc++)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(quad src/Chassis.cpp src/Leg.cpp src/Robot.cpp src/Servo.cpp src/SSC32U.cpp)
target_include_directories(quad PUBLIC include)
target_link_libraries(quad Eigen3::Eigen -static)
target_compile_features(quad PRIVATE cxx_std_20)