
function (example_scope_guard)
    set (name "ScopeGuard")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/ScopeGuardExample.cpp")

    add_executable (${name} ${src})
endfunction ()

#################################################################

example_scope_guard ()
