cmake_minimum_required(VERSION 3.2)

# Project description
project(
    urdfdomcpp
    VERSION 1.0
    DESCRIPTION "A non-ROS version of urdfdom"
    LANGUAGES CXX)

# Use conan to gather dependencies if enabled
include(cmake/Conan.cmake)

# Otherwise fallback to system dependencies
if(NOT ${PROJECT_NAME}_ENABLE_CONAN)
    find_package(tinyxml2)
endif()

option(ENABLE_TESTING "Enable test compilation" OFF)
if(ENABLE_TESTING)
    enable_testing()
endif()

# Process the src and apps directories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)

# Force the generation of a compile_commands.json file to provide autocompletion for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "" FORCE)
