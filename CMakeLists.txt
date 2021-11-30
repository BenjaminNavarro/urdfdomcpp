cmake_minimum_required(VERSION 3.10)

# Project description
project(
    urdfdomcpp
    VERSION 1.0.1
    DESCRIPTION "A non-ROS version of urdfdom"
    LANGUAGES CXX)

# Set the build type to release if not specified by the user
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add static analyzers support
include(cmake/StaticAnalyzers.cmake)

option(ENABLE_TESTING "Enable test compilation" OFF)
if(ENABLE_TESTING)
    enable_testing()
endif()

option(USE_SYSTEM_DEPENDENCIES "Use system dependencies instead of Conan ones" OFF)
if(NOT USE_SYSTEM_DEPENDENCIES)
    include(cmake/Conan.cmake)
endif()

find_package(tinyxml2 REQUIRED)
if(ENABLE_TESTING)
    find_package(GTest REQUIRED)
endif()

# Process the src and apps directories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)

# Force the generation of a compile_commands.json file to provide autocompletion for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "" FORCE)
