cmake_minimum_required (VERSION 2.8)
project ("embxx")
 
set (EXTERNALS_DIR "${CMAKE_SOURCE_DIR}/external")
find_package (Git REQUIRED)
 
execute_process (
    COMMAND ${GIT_EXECUTABLE} submodule update --init
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
            
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")         

include (AR_AllExtras)
    
ar_add_cpp11_support ()
ar_define_doxygen_tgt ("doxygen" "${CMAKE_SOURCE_DIR}/doc/doxygen.conf")
ar_set_default_compiler_options ()

include_directories (
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/external/cxxtest")
    
if (NOT DEFINED CROSS_COMPILE AND NOT DEFINED NO_UNIT_TEST)
    set (CXXTEST_INCLUDE_DIR "${EXTERNALS_DIR}/cxxtest")
    find_package (CxxTest)
    if (CXXTEST_FOUND)
        enable_testing ()
    endif ()
endif ()

add_subdirectory (module)

install (
    DIRECTORY "${CMAKE_SOURCE_DIR}/embxx"
    DESTINATION "${CMAKE_BINARY_DIR}/install")

