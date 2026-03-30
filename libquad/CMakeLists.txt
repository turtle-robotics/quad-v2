option(BUILD_TESTING "..." OFF)

add_library(libquad STATIC src/Chassis.cpp src/Leg.cpp)
target_include_directories(libquad PUBLIC include)
target_link_libraries(libquad PUBLIC Eigen3::Eigen ModernRobotics yaml-cpp::yaml-cpp)
target_compile_options(libquad PUBLIC -Werror)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()