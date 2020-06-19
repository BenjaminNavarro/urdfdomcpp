cmake_minimum_required(VERSION 3.2)

# Project description
project(
    urdfdomcpp
    VERSION 1.0
    DESCRIPTION "A non-ROS version of urdfdom"
    LANGUAGES CXX)

# Use conan to gather dependencies
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

option(ENABLE_TESTING "Enable test compilation" OFF)
if(ENABLE_TESTING)
    enable_testing()
endif()

option(BUILD_SHARED_LIBRARY "Build the library as a shared one" ON)

# Process the src and apps directories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)

# Force the generation of a compile_commands.json file to provide autocompletion for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "" FORCE)
