# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_timer_mgr)
    set (test_suite_name "TimerMgr")
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

function (test_generic)
    set (test_suite_name "Generic")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_character)
    set (test_suite_name "Character")
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

function (test_gpio)
    set (test_suite_name "Gpio")
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

test_timer_mgr()
test_generic()
test_character()
test_gpio()

endif ()
