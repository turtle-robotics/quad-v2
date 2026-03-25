# Add the test executable
add_executable(kinematics kinematics.cpp)

# Link against GTest libraries (specifically gtest_main provides the main() entry point)
target_link_libraries(kinematics PRIVATE GTest::gtest_main GTest::gtest)

# Discover GTest tests automatically and add them to CTest
# This command uses GTest's features to list tests and registers each one with CTest
gtest_discover_tests(kinematics)