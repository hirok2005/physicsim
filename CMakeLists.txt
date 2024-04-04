cmake_minimum_required(VERSION 3.16)
project(physicsim LANGUAGES CXX)

enable_testing()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Check if SFML is already installed
find_package(SFML QUIET COMPONENTS graphics window)

# If SFML not found, use FetchContent to download SFML
if(NOT SFML_FOUND)
  include(FetchContent)
  FetchContent_Declare(SFML
      GIT_REPOSITORY https://github.com/SFML/SFML.git
      GIT_TAG 2.6.x
  )
  FetchContent_MakeAvailable(SFML)  
endif()

# Set SFML_DIR to the directory containing SFMLConfig.cmake
if(SFML_FOUND)
  set(SFML_DIR ${SFML_DIR}/lib/cmake/SFML)
endif()

# Create the physicsim library
add_library(physicsim 
    "include/physicsim/physicsim.hpp" 
    "src/render.cpp"
    "src/matrix.cpp"
    "src/rigidbody.cpp"
    "src/world.cpp"
  "src/collisions.cpp")

target_include_directories(physicsim PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_link_libraries(physicsim PRIVATE sfml-graphics sfml-window)
target_compile_features(physicsim PRIVATE cxx_std_23)

add_subdirectory(tests)
