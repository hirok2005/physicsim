cmake_minimum_required(VERSION 3.16)

# Add the executable for the test
add_executable(tester "test1.cpp" )

# Include directories for the tester executable
target_include_directories(tester PRIVATE "${CMAKE_SOURCE_DIR}/include")

# Link the executable with the physicsim library and SFML libraries
target_link_libraries(tester PRIVATE physicsim sfml-graphics sfml-window)

# Set the C++ standard to C++17
target_compile_features(tester PRIVATE cxx_std_17)

# Add the test with the executable and the command-line arguments
add_test(NAME physicsim.funcs.foo COMMAND tester foo)
