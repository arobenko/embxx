# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

set (TEST_OBJECT_LIB_NAME "TestObject")
#################################################################

function (lib_test_object)
    set (name "${TEST_OBJECT_LIB_NAME}")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/TestObject.cpp")

    add_library (${name} STATIC ${src})
endfunction ()

#################################################################

function (test_static_queue)
    set (test_suite_name "StaticQueue")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link
        "${TEST_OBJECT_LIB_NAME}")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

lib_test_object()
test_static_queue()

endif ()
