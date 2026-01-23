cmake_minimum_required(VERSION 4.0)
project(
  TurtleQUAD
  VERSION 1.0
  LANGUAGES CXX)

include(FetchContent)

set(remote_user quad)
set(remote_host quad2.local)

set(CMAKE_CXX_FLAGS -static-libstdc++)

find_package(Eigen3 3.4...5 REQUIRED NO_MODULE)

# FetchContent_Declare(
#   Eigen3
#   GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#   GIT_TAG 5.0.1
# )
# FetchContent_MakeAvailable(Eigen3)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

set(SDL_STATIC true)
FetchContent_Declare(
  SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-3.4.0
)
FetchContent_MakeAvailable(SDL3)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20260116
)
FetchContent_MakeAvailable(moteus)

FetchContent_Declare(
  pi3hat
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
  GIT_TAG 0.1-20260117
)
FetchContent_MakeAvailable(pi3hat)

add_executable(quad src/Robot.cpp src/Chassis.cpp src/Leg.cpp)

target_include_directories(quad PUBLIC include)
target_link_libraries(quad Eigen3::Eigen yaml-cpp::yaml-cpp SDL3::SDL3-static moteus::cpp pi3hat::pi3hat)# -static)
# target_compile_features(quad PRIVATE cxx_std_20)
# target_compile_definitions(quad PRIVATE QUAD_V1=1)

# separate motor test executable
add_executable(motor_test src/position_control.cpp)
target_include_directories(motor_test PUBLIC include)
target_link_libraries(motor_test PRIVATE Eigen3::Eigen yaml-cpp::yaml-cpp moteus::cpp pi3hat::pi3hat)

# Upload to robot after build
add_custom_command(TARGET quad POST_BUILD COMMAND rsync -z $<TARGET_FILE:quad> "${remote_user}@${remote_host}:/home/${remote_user}/quad")
add_custom_command(TARGET quad POST_BUILD COMMAND rsync -rz configs "${remote_user}@${remote_host}:/home/${remote_user}/")
add_custom_command(TARGET motor_test POST_BUILD COMMAND rsync -z $<TARGET_FILE:motor_test> "${remote_user}@${remote_host}:/home/${remote_user}/motor_test")