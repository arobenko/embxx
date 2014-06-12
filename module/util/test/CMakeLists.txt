# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_assert)
    set (test_suite_name "Assert")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_scope_guard)
    set (test_suite_name "ScopeGuard")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_size_to_type)
    set (test_suite_name "SizeToType")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_allocators)
    set (test_suite_name "Allocators")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_aligned_union)
    set (test_suite_name "AlignedUnion")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_tuple)
    set (test_suite_name "Tuple")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_integral_promotion)
    set (test_suite_name "IntegralPromotion")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_event_loop)
    set (test_suite_name "EventLoop")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link
        "pthread")
        
    set (extra_flags
        "-Wl,--no-as-needed") # Workaround for some compiler bug in gcc-4.8 64bit

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
    set_target_properties(${name} PROPERTIES LINK_FLAGS ${extra_flags})
    
endfunction ()

#################################################################

function (test_static_function)
    set (test_suite_name "StaticFunction")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_static_pool_allocator)
    set (test_suite_name "StaticPoolAllocator")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}Test")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release") 
test_assert()
endif ()

test_scope_guard()
test_size_to_type()
test_allocators()
test_aligned_union()
test_tuple()
test_integral_promotion()
test_event_loop()
test_static_function()
test_static_pool_allocator()

endif ()
