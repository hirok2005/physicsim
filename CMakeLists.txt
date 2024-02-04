cmake_minimum_required(VERSION 3.16)
project(physicsim LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

# Create the physicsim library
add_library(physicsim 
    "include/physicsim/physicsim.hpp" 
    "src/physicsim.cpp")

target_include_directories(physicsim PRIVATE ${SFML_SOURCE_DIR}/include)
target_link_libraries(physicsim PRIVATE sfml-graphics sfml-window)
target_compile_features(physicsim PRIVATE cxx_std_17)