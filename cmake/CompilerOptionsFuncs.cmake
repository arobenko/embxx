macro (_ar_add_compiler_flags cmake_flags options)
    foreach (flag ${cmake_flags})
        set (new_flags ${${flag}} ${options})
        string (REPLACE ";" " " ${flag} "${new_flags}")
    endforeach ()
endmacro ()

macro (ar_add_cxx_flags)
          _ar_add_compiler_flags ("CMAKE_CXX_FLAGS" "${ARGN}")
endmacro ()

macro (ar_add_c_flags)
     _ar_add_compiler_flags ("CMAKE_C_FLAGS" "${ARGN}")
endmacro ()

macro (ar_add_c_cxx_flags)
    ar_add_c_flags (${ARGN})
    ar_add_cxx_flags (${ARGN})
endmacro ()

macro (ar_set_default_compiler_options)
    if (UNIX)
        set (extra_c_cxx_flags 
            "-Wall"
            "-Wextra"
            "-Werror")
        ar_add_c_cxx_flags (${extra_c_cxx_flags})
        
        set (extra_cxx_flags
            "-Woverloaded-virtual")
       
        ar_add_cxx_flags (${extra_c_cxx_flags})
    endif ()
endmacro ()

macro (ar_add_cpp11_support)
    if (UNIX)
        ar_add_cxx_flags("--std=c++0x")
    endif ()
endmacro ()

macro (ar_disable_exceptions)
    if (UNIX)
        ar_add_cxx_flags("-fno-exceptions -fno-unwind-tables")
    endif ()
endmacro ()

macro (ar_disable_rtti)
    if (UNIX)
        ar_add_cxx_flags("-fno-rtti")
    endif ()
endmacro ()

macro (ar_disable_stdlib)
    add_definitions(-DNOSTDLIB)
    if (UNIX)
        ar_add_c_cxx_flags("-nostdlib")
    endif ()
endmacro ()

