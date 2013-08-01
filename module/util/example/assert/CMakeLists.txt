
function (example_default_assert)
    set (name "DefaultAssert")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/DefaultAssert.cpp")

    add_executable (${name} ${src})
    
    install (
        TARGETS ${name} 
        DESTINATION "${EXAMPLES_DIR}/${COMPONENT_NAME}")
endfunction ()

#################################################################

function (example_exception_assert)
    set (name "ExceptionAssert")
    
    set (src "${CMAKE_CURRENT_SOURCE_DIR}/ExceptionAssert.cpp")

    add_executable (${name} ${src})

    install (
        TARGETS ${name} 
        DESTINATION "${EXAMPLES_DIR}/${COMPONENT_NAME}")
            
endfunction ()

#################################################################

example_default_assert ()
example_exception_assert ()
