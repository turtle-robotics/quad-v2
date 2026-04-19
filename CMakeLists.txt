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

add_compile_options(-static-libstdc++ -std=c++20 -Wextra -Wno-psabi -Wno-deprecated -pedantic -w)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20260121
)

FetchContent_Declare(
  pi3hat
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
  GIT_TAG 0.1-20260117
)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
FetchContent_MakeAvailable(yaml-cpp)
# find_package(HDF5 COMPONENTS C CXX REQUIRED)
# find_package(GTest REQUIRED)
FetchContent_MakeAvailable(moteus)

add_subdirectory(libquad)

add_executable(quad src/main.cpp src/Robot.cpp src/Teleop.cpp)


target_include_directories(quad PUBLIC include)
target_compile_options(quad PUBLIC -Werror)
target_link_libraries(quad PUBLIC libquad Eigen3::Eigen yaml-cpp::yaml-cpp moteus::cpp spdlog::spdlog_header_only)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  FetchContent_MakeAvailable(pi3hat)
  target_link_libraries(quad PUBLIC pi3hat::pi3hat)
endif()

# Upload code & configs to robot (make deploy)
add_custom_target(deploy COMMAND rsync -rz $<TARGET_FILE:quad> ${CMAKE_CURRENT_SOURCE_DIR}/configs "${remote_user}@${remote_host}:/home/${remote_user}/")

add_dependencies(deploy quad)