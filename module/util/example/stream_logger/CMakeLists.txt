
function (example_simple_log)
    set (name "SimpleLogExample")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/SimpleLogExample.cpp")

    add_executable (${name} ${src})
endfunction ()

#################################################################

function (example_mutlti_threaded_log)
    set (name "MultiThreadedLogExample")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/MultiThreadedLogExample.cpp")

    add_executable (${name} ${src})
    target_link_libraries(${name} "pthread")
endfunction ()

#################################################################

example_simple_log ()
example_mutlti_threaded_log ()
