cmake_minimum_required(VERSION 3.16)

# Add the executable for the test
add_executable(testMat "testMatrix.cpp")
add_executable(testSim "testSim.cpp")

target_include_directories(testMat PRIVATE "${CMAKE_SOURCE_DIR}/include")
target_include_directories(testSim PRIVATE "${CMAKE_SOURCE_DIR}/include")

# Link the executable with the physicsim library and SFML libraries
target_link_libraries(testMat PRIVATE physicsim sfml-graphics sfml-window)
target_link_libraries(testSim PRIVATE physicsim sfml-graphics sfml-window)

# Set the C++ standard to C++23
target_compile_features(testMat PRIVATE cxx_std_23)
target_compile_features(testSim PRIVATE cxx_std_23)

# Add the tests with the executable and the command-line arguments
# Note: Assuming you want to run two separate tests for matrix and update
add_test(NAME physicsim.funcs.matrix COMMAND testMat)
add_test(NAME physicsim.funcs.sim COMMAND testSim)
