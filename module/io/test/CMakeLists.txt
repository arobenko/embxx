# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_iosimple)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/iosimple.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.IoSimple")

    set (runner "testRunner.cpp")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

test_iosimple()

endif ()
