cmake_minimum_required(VERSION 3.28.3)
project(
  TurtleQUAD
  VERSION 1.0
  LANGUAGES CXX)

include(FetchContent)

# set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
# set(CMAKE_CXX_FLAGS -static-libstdc++)

find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
find_package(yaml-cpp 0 REQUIRED NO_MODULE)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20250925
)
FetchContent_Declare(
  SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-3.2.24
)
FetchContent_MakeAvailable(moteus)
FetchContent_MakeAvailable(SDL3)

add_executable(quad src/main.cpp src/Robot.cpp src/Chassis.cpp src/Leg.cpp)
target_include_directories(quad PUBLIC include)
target_link_libraries(quad Eigen3::Eigen yaml-cpp::yaml-cpp SDL3::SDL3 moteus::cpp) # -static)
# target_compile_features(quad PRIVATE cxx_std_20)
# target_compile_definitions(quad PRIVATE QUAD_V1=1)

add_executable(sdl_test src/sdl_test.cpp)
target_link_libraries(sdl_test PRIVATE SDL3::SDL3)