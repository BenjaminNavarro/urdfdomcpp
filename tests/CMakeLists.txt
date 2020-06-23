if(ENABLE_TESTING)
    execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
    execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)

    # unit test to fix geometry problems
    set(tests
        urdf_double_convert
        urdf_unit_test
        urdf_version_test
    )

    #################################################
    # Build all the tests
    foreach(test ${tests})
        add_executable(${test} ${test}.cpp)

        target_link_libraries(${test} CONAN_PKG::gtest urdfdomcpp)
        if (UNIX)
            target_link_libraries(${test} pthread)
        endif()

        add_test(
            NAME ${test}
            COMMAND ${test}
                --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}.xml
        )

        set_tests_properties(${test} PROPERTIES TIMEOUT 240 ENVIRONMENT LC_ALL=C)

        add_test(
            NAME ${test}_locale
            COMMAND ${test}
                --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}_locale.xml)

        set_tests_properties(${test}_locale PROPERTIES TIMEOUT 240 ENVIRONMENT LC_ALL=nl_NL.UTF-8)
    endforeach()

endif()