# Automatically downloads conan.cmake if not already present
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD   https://raw.githubusercontent.com/conan-io/cmake-conan/0.17.0/conan.cmake
                    ${CMAKE_BINARY_DIR}/conan.cmake)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_add_remote(
    NAME bnavarro
    URL https://navarrob.jfrog.io/artifactory/api/conan/navarrob-conan
)

if(ENABLE_TESTING)
    set(conan_build_tests True)
else()
    set(conan_build_tests False)
endif()

conan_cmake_run(
    CONANFILE conan/conanfile.py
    GENERATORS cmake_find_package
    BUILD missing
    OPTIONS urdfdomcpp:build_tests=${conan_build_tests}
)

set(conan_build_tests)