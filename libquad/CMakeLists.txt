option(BUILD_TESTING "..." OFF)

set(PYBIND11_FINDPYTHON ON)

find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG)

add_compile_options(-static-libstdc++ -std=c++20 -Wextra -Wno-psabi -Wno-deprecated -pedantic -w)

add_library(libquad STATIC src/Chassis.cpp src/Leg.cpp)
target_include_directories(libquad PUBLIC include)
target_link_libraries(libquad PUBLIC Eigen3::Eigen)
target_compile_options(libquad PUBLIC -Werror)
target_compile_definitions(
  libquad
  PUBLIC
  EIGEN_TRANSFORM_PLUGIN="eigen_plugins/transform.hpp"
  EIGEN_MATRIXBASE_PLUGIN="eigen_plugins/matrixbase.hpp"
)

if (TARGET pybind11::module)
  pybind11_add_module(libquad_python src/python_bindings.cpp)
  set_target_properties(libquad_python PROPERTIES OUTPUT_NAME "libquad")

  target_link_libraries(libquad_python PUBLIC libquad)
  install(TARGETS libquad_python DESTINATION .)
endif (TARGET pybind11::module)

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(leg_tests test/leg_tests.cpp)
  target_link_libraries(leg_tests libquad Eigen3::Eigen GTest::gtest_main)
  gtest_discover_tests(leg_tests)
endif()