add_subdirectory(tests)

add_library(libquad STATIC src/Chassis.cpp src/Leg.cpp)
target_include_directories(libquad PUBLIC include)
target_link_libraries(libquad PUBLIC Eigen3::Eigen ModernRobotics)
target_compile_options(libquad PUBLIC -Werror)