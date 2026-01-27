cmake_minimum_required(VERSION 4.0)
project(
  TurtleQUAD
  VERSION 2.0
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

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20260121
)
FetchContent_MakeAvailable(moteus)

FetchContent_Declare(
  pi3hat
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
  GIT_TAG 0.1-20260117
)
FetchContent_MakeAvailable(pi3hat)

add_executable(quad src/main.cpp src/Robot.cpp src/Chassis.cpp src/Leg.cpp src/Teleop.cpp)

target_include_directories(quad PUBLIC include)
target_link_libraries(quad PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp moteus::cpp pi3hat::pi3hat)# -static)
target_compile_features(quad PRIVATE cxx_std_20)
# target_compile_definitions(quad PRIVATE QUAD_V1=1)

# Upload to robot after build
add_custom_command(TARGET quad POST_BUILD COMMAND rsync -z $<TARGET_FILE:quad> "${remote_user}@${remote_host}:/home/${remote_user}/quad")
add_custom_command(TARGET quad POST_BUILD COMMAND rsync -rz configs "${remote_user}@${remote_host}:/home/${remote_user}/")

get_target_property(OUT quad LINK_LIBRARIES)
message(STATUS ${OUT})
