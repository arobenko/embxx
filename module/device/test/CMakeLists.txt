# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_device_op_queue)
    set (test_suite_name "DeviceOpQueue")
    if ((NOT Boost_FOUND) OR (NOT Boost_SYSTEM_LIBRARY))
        message (WARNING "Skipping unittests for ${test_suite_name}, due to missing boost")
        return ()
    endif()
    
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")

    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_id_device_char_adapter)
    set (test_suite_name "IdDeviceCharAdapter")
    if ((NOT Boost_FOUND) OR (NOT Boost_SYSTEM_LIBRARY))
        message (WARNING "Skipping unittests for ${test_suite_name}, due to missing boost")
        return ()
    endif()
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

test_device_op_queue()
test_id_device_char_adapter()

endif ()
