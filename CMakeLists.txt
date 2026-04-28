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
  GIT_TAG yaml-cpp-0.9.0
)

FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG 0.1-20260424
)

FetchContent_Declare(
  pi3hat
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
  GIT_TAG 0.1-20260117
)

find_package(Eigen3 REQUIRED)
# find_package(spdlog REQUIRED)
FetchContent_MakeAvailable(yaml-cpp)
FetchContent_MakeAvailable(moteus)

add_subdirectory(libquad)

# Only build the executable with arm64 architecture
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  FetchContent_MakeAvailable(pi3hat)
  
  add_executable(quad src/main.cpp src/Robot.cpp src/HID.cpp)
  target_include_directories(quad PUBLIC include)
  target_compile_options(quad PUBLIC -Werror)
  target_link_libraries(quad PUBLIC -static libquad Eigen3::Eigen yaml-cpp::yaml-cpp moteus::cpp pi3hat::pi3hat)

  # Upload code & configs to robot (make deploy)
  add_custom_target(deploy COMMAND rsync -rz $<TARGET_FILE:quad> ${CMAKE_CURRENT_SOURCE_DIR}/configs "${remote_user}@${remote_host}:/home/${remote_user}/")

  add_dependencies(deploy quad)
endif()

find_package(Doxygen OPTIONAL)

if(Doxygen_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/assets/turtle.svg ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg @ONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/banner.svg ${CMAKE_CURRENT_BINARY_DIR}/banner.svg @ONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md ${CMAKE_CURRENT_BINARY_DIR}/README.md @ONLY)
  # configure_file(${CMAKE_CURRENT_SOURCE_DIR}/assets/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/assets/libstdc++.tag ${CMAKE_CURRENT_BINARY_DIR}/libstdc++.tag @ONLY)
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_GENERATE_LATEX YES)
  set(DOXYGEN_GENERATE_TAGFILE ${CMAKE_CURRENT_BINARY_DIR}/quad.cfg)
  set(DOXYGEN_TAGFILES "${CMAKE_CURRENT_BINARY_DIR}/libstdc++.tag=https://gcc.gnu.org/onlinedocs/libstdc++/latest-doxygen")
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXCLUDE_SYMBOLS YAML::*)
  set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/docs")
  set(DOXYGEN_PROJECT_LOGO "${CMAKE_BINARY_DIR}/docs")
  set(DOXYGEN_HTML_COLORSTYLE_HUE 0)
  set(DOXYGEN_HTML_COLORSTYLE_SAT 255)
  set(DOXYGEN_HTML_COLORSTYLE_GAMMA 74)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${CMAKE_CURRENT_BINARY_DIR}/README.md)
  set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_BINARY_DIR}/banner.svg)
  set(DOXYGEN_PROJECT_LOGO ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg)
  set(DOXYGEN_PROJECT_ICON ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg)
  doxygen_add_docs(quaddocs ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/libquad ${CMAKE_CURRENT_BINARY_DIR}/README.md COMMENT "Generating API documentation")
endif()