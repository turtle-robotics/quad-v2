cmake_minimum_required(VERSION 4.0)
project(
  TurtleQUAD
  VERSION 2.0
  LANGUAGES CXX)

include(FetchContent)

enable_testing()

# Remote deployment settings
set(remote_user quad)
set(remote_host quad2.local)

add_compile_options(-static-libstdc++ -std=c++20 -Wextra -Wno-psabi -Wno-deprecated -pedantic)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.1
)
FetchContent_MakeAvailable(Eigen3)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

# FetchContent_Declare(
#   spdlog
#   GIT_REPOSITORY https://github.com/gabime/spdlog.git
#   GIT_TAG v1.17.0
# )
# FetchContent_MakeAvailable(spdlog)

find_package(HDF5 COMPONENTS C CXX)
find_package(GTest REQUIRED)

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

FetchContent_Declare(
  ModernRobotics
  GIT_REPOSITORY https://github.com/Ian118/ModernRobotics.git
  GIT_TAG master
)
FetchContent_MakeAvailable(ModernRobotics)

add_subdirectory(libquad)

add_executable(quad src/main.cpp src/Robot.cpp src/Teleop.cpp)

target_include_directories(quad PUBLIC include)
# spdlog hdf5::hdf5_cpp
target_link_libraries(quad PUBLIC libquad Eigen3::Eigen yaml-cpp::yaml-cpp moteus::cpp pi3hat::pi3hat)
target_compile_options(quad PUBLIC -Werror)

# Upload code & configs to robot (make deploy)
add_custom_target(deploy COMMAND rsync -rz $<TARGET_FILE:quad> ${CMAKE_CURRENT_SOURCE_DIR}/configs "${remote_user}@${remote_host}:/home/${remote_user}/")

add_dependencies(deploy quad)