option(BUILD_TESTING "..." OFF)

add_library(libquad STATIC src/Chassis.cpp src/Leg.cpp)
target_include_directories(libquad PUBLIC include)
target_link_libraries(libquad PUBLIC Eigen3::Eigen ModernRobotics yaml-cpp::yaml-cpp)
target_compile_options(libquad PUBLIC -Werror)
target_compile_definitions(
  libquad
  PUBLIC
  EIGEN_TRANSFORM_PLUGIN="eigen_plugins/transform.hpp"
  EIGEN_MATRIXBASE_PLUGIN="eigen_plugins/matrixbase.hpp"
)

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(leg_tests test/leg_tests.cpp)
  target_link_libraries(leg_tests libquad Eigen3::Eigen GTest::gtest_main)
  gtest_discover_tests(leg_tests)
endif()