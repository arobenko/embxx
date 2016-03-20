# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_access)
    set (test_suite_name "access")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_std_streambuf_access)
    set (test_suite_name "std_streambuf_access")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_write_queue)
    set (test_suite_name "WriteQueue")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")

    if ((NOT Boost_INCLUDE_DIRS) OR (NOT Boost_SYSTEM_LIBRARY))
      find_package(Boost COMPONENTS system REQUIRED)
    endif()
    if(Boost_FOUND)
      include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")
    endif()

    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_out_stream_buf)
    set (test_suite_name "OutStreamBuf")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    if ((NOT Boost_INCLUDE_DIRS) OR (NOT Boost_SYSTEM_LIBRARY))
      find_package(Boost COMPONENTS system REQUIRED)
    endif()
    if(Boost_FOUND)
      include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")
    endif()

    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_out_stream)
    set (test_suite_name "OutStream")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    if ((NOT Boost_INCLUDE_DIRS) OR (NOT Boost_SYSTEM_LIBRARY))
      find_package(Boost COMPONENTS system REQUIRED)
    endif()
    if(Boost_FOUND)
      include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")
    endif()

    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_in_stream_buf)
    set (test_suite_name "InStreamBuf")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    if ((NOT Boost_INCLUDE_DIRS) OR (NOT Boost_SYSTEM_LIBRARY))
      find_package(Boost COMPONENTS system REQUIRED)
    endif()
    if(Boost_FOUND)
      include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")
    endif()

    set (link
        "${Boost_SYSTEM_LIBRARY}"
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

test_access()
test_std_streambuf_access()
test_write_queue()
test_out_stream_buf()
test_out_stream()
test_in_stream_buf()

endif ()
