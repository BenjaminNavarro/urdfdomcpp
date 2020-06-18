add_executable(check_urdf ${CMAKE_CURRENT_SOURCE_DIR}/check_urdf/main.cpp)
target_link_libraries(check_urdf PUBLIC urdfdomcpp)

add_executable(urdf_to_graphiz ${CMAKE_CURRENT_SOURCE_DIR}/urdf_to_graphiz/main.cpp)
target_link_libraries(urdf_to_graphiz PUBLIC urdfdomcpp)