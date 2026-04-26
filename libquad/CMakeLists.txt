option(BUILD_TESTING "..." OFF)

find_package(Eigen3 REQUIRED)
find_package(Doxygen OPTIONAL)

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

if(Doxygen_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../assets/turtle.svg ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg @ONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../banner.svg ${CMAKE_CURRENT_BINARY_DIR}/banner.svg @ONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../README.md ${CMAKE_CURRENT_BINARY_DIR}/README.md @ONLY)
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_GENERATE_LATEX YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/docs")
  set(DOXYGEN_PROJECT_LOGO "${CMAKE_BINARY_DIR}/docs")
  set(DOXYGEN_HTML_COLORSTYLE_HUE 0)
  set(DOXYGEN_HTML_COLORSTYLE_SAT 255)
  set(DOXYGEN_HTML_COLORSTYLE_GAMMA 74)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${CMAKE_CURRENT_BINARY_DIR}/README.md)
  set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_BINARY_DIR}/banner.svg)
  set(DOXYGEN_PROJECT_LOGO ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg)
  set(DOXYGEN_PROJECT_ICON ${CMAKE_CURRENT_BINARY_DIR}/turtle.svg)
  doxygen_add_docs(libquaddocs ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/README.md COMMENT "Generating API documentation")
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(leg_tests test/leg_tests.cpp)
  target_link_libraries(leg_tests libquad Eigen3::Eigen GTest::gtest_main)
  gtest_discover_tests(leg_tests)
endif()