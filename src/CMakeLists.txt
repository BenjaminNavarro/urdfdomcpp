file(GLOB urdfdomcpp_FILES ${CMAKE_CURRENT_SOURCE_DIR}/urdf_parser/*.cpp)

# will be a shared or static library depending on BUILD_SHARED_LIBS
add_library(urdfdomcpp ${urdfdomcpp_FILES})

target_include_directories(urdfdomcpp PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_compile_features(urdfdomcpp PUBLIC cxx_std_11)

target_link_libraries(urdfdomcpp PUBLIC CONAN_PKG::tinyxml2)

install(
    TARGETS urdfdomcpp
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/include 
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)