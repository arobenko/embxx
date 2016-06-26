cmake_minimum_required (VERSION 2.8)
project ("embxx")
 
set (EXTERNALS_DIR "${CMAKE_SOURCE_DIR}/external")
find_package (Git REQUIRED)
 
execute_process (
    COMMAND ${GIT_EXECUTABLE} submodule update --init
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
            
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")         

include (AR_AllExtras)
    
embxx_add_cpp11_support ()
embxx_define_doxygen_tgt ("doxygen" "${CMAKE_SOURCE_DIR}/doc/doxygen.conf")
embxx_set_default_compiler_options ()

include_directories (
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/external/cxxtest")
    
if (NOT DEFINED CMAKE_CROSSCOMPILE AND "${NO_UNIT_TESTS}" STREQUAL "")
    set (CXXTEST_INCLUDE_DIR "${EXTERNALS_DIR}/cxxtest")
    find_package (CxxTest)
    if (CXXTEST_FOUND)
        enable_testing ()
    endif ()
    
    find_package(Boost COMPONENTS system)
    if(Boost_FOUND)
        include_directories(SYSTEM "${Boost_INCLUDE_DIRS}")
    endif()
endif ()

set (INSTALL_DIR "${CMAKE_BINARY_DIR}/install")
set (EXAMPLES_DIR "${INSTALL_DIR}/example")

install (
    DIRECTORY "${CMAKE_SOURCE_DIR}/embxx"
    DESTINATION "${INSTALL_DIR}/include")

add_subdirectory (module)

