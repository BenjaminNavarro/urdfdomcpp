file(GLOB urdfdomcpp_FILES ${CMAKE_CURRENT_SOURCE_DIR}/urdf_parser/*.cpp)

add_library(urdfdomcpp SHARED ${urdfdomcpp_FILES})

target_include_directories(
    urdfdomcpp 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> 
        $<INSTALL_INTERFACE:include>
)

target_compile_features(urdfdomcpp PUBLIC cxx_std_11)

target_link_libraries(urdfdomcpp PRIVATE CONAN_PKG::tinyxml2)

install(
    TARGETS urdfdomcpp
    EXPORT urdfdomcppConfig
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
)

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/include 
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
    EXPORT urdfdomcppConfig 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake
)