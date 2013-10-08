# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_assert)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/Assert.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.Assert")

    set (runner "AssertTestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_scope_guard)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/ScopeGuard.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.ScopeGuard")

    set (runner "ScopeGuardTestRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_size_to_type)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/SizeToType.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.SizeToType")

    set (runner "SizeToTypeRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_allocators)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/Allocators.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.Allocators")

    set (runner "AllocatorsRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_aligned_union)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/AlignedUnion.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.AlignedUnion")

    set (runner "AlignedUnionRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_tuple)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/Tuple.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.Tupple")

    set (runner "TupleRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_integral_promotion)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/IntegralPromotion.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.IntegralPromotion")

    set (runner "IntegralPromotionRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_event_loop)
    set (test_suite_name "EventLoop")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_static_function)
    set (test_suite_name "StaticFunction")
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/${test_suite_name}.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.${test_suite_name}")

    set (runner "${test_suite_name}TestRunner.cpp")
    
    set (link
        "pthread")

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

test_assert()
test_scope_guard()
test_size_to_type()
test_allocators()
test_aligned_union()
test_tuple()
test_integral_promotion()
test_event_loop()
test_static_function()

endif ()
