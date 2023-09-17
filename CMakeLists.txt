cmake_minimum_required(VERSION 3.22)
project(quad LANGUAGES CXX)

include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif ()
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)

if ("${BUILD_TYPE_UPPER}" STREQUAL "RELEASE")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build Type" FORCE)

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math")
elseif ("${BUILD_TYPE_UPPER}" STREQUAL "DEBUG")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build Type" FORCE)

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

  add_definitions(-DDEBUG)
endif ()

add_library(quad_lib INTERFACE)
target_include_directories(quad_lib
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/exe>
    $<INSTALL_INTERFACE:include>
)


add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third-party/libzmq")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third-party/cppzmq")
find_package(cppzmq)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
target_link_libraries(quad_lib INTERFACE
    Eigen3::Eigen
    cppzmq
)

option(BUILD_EXE "Build Executables" OFF)

# examples
if (BUILD_EXE)
  # add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third-party/matplotlib-cpp")

  add_executable(test exe/test.cpp)
  target_link_libraries(test PRIVATE quad_lib)
  set_target_properties(test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

endif ()

install(FILES
  "${PROJECT_SOURCE_DIR}/quad_lib.hpp"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
