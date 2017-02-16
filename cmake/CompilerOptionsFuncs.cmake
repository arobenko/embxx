# Copyright 2013 (C). Alex Robenko. All rights reserved.

# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

macro (_embxx_add_compiler_flags cmake_flags options)
    foreach (flag ${cmake_flags})
        set (new_flags ${${flag}} ${options})
        string (REPLACE ";" " " ${flag} "${new_flags}")
    endforeach ()
endmacro ()

macro (embxx_add_cxx_flags)
          _embxx_add_compiler_flags ("CMAKE_CXX_FLAGS" "${ARGN}")
endmacro ()

macro (embxx_add_c_flags)
     _embxx_add_compiler_flags ("CMAKE_C_FLAGS" "${ARGN}")
endmacro ()

macro (embxx_add_asm_flags)
     _embxx_add_compiler_flags ("CMAKE_ASM_FLAGS" "${ARGN}")
endmacro ()

macro (embxx_add_c_cxx_flags)
    embxx_add_c_flags (${ARGN})
    embxx_add_cxx_flags (${ARGN})
endmacro ()

macro (embxx_add_asm_c_cxx_flags)
    embxx_add_asm_flags (${ARGN})
    embxx_add_c_flags (${ARGN})
    embxx_add_cxx_flags (${ARGN})
endmacro ()

macro (embxx_set_default_compiler_options)
    if (CMAKE_COMPILER_IS_GNUCC)
        set (extra_c_cxx_flags 
            "-Wall"
            "-Wextra"
            "-Werror"
            "-Wcast-align"
            "-Wcast-qual"
            "-Wmissing-include-dirs"
            "-Wlogical-op"
            "-Wstrict-null-sentinel"
            "-Wredundant-decls"
            "-Wno-unknown-pragmas"
            "-Wundef" 
            "-Wunused"
            "-Wshadow"
            "-fdiagnostics-show-option")
        embxx_add_c_cxx_flags (${extra_c_cxx_flags})
        
        set (extra_cxx_flags
            "-Woverloaded-virtual"
            "-Wno-unused-local-typedefs"
            "-Wctor-dtor-privacy"
            "-Wnoexcept")
       
        embxx_add_cxx_flags (${extra_cxx_flags})
    endif ()
endmacro ()

macro (_embxx_add_cpp11_support_direct)
    if (CMAKE_COMPILER_IS_GNUCC OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
        embxx_add_cxx_flags("--std=c++0x")
    else ()
        message (WARNING "Unknown compiler, manual set of C++11 support may be required!")
    endif ()
endmacro ()

macro (embxx_add_cpp11_support)
    while (TRUE)
        if (CMAKE_VERSION VERSION_LESS 3.1)
            _embxx_add_cpp11_support_direct()
            break()
        endif ()
        
        if ((DEFINED CMAKE_CROSSCOMPILING) AND (CMAKE_VERSION VERSION_LESS 3.6))
            _embxx_add_cpp11_support_direct()
            break()
        endif ()
        
        set (CMAKE_CXX_STANDARD   11)
        set (CMAKE_CXX_EXTENSIONS OFF)
        break()
    endwhile ()
endmacro ()

macro (embxx_disable_exceptions)
    if (CMAKE_COMPILER_IS_GNUCC)
        embxx_add_cxx_flags("-fno-exceptions -fno-unwind-tables")
    endif ()
endmacro ()

macro (embxx_disable_rtti)
    if (CMAKE_COMPILER_IS_GNUCC)
        embxx_add_cxx_flags("-fno-rtti")
    endif ()
endmacro ()

macro (embxx_disable_stdlib)
    add_definitions(-DNOSTDLIB)
    if (CMAKE_COMPILER_IS_GNUCC)
        embxx_add_c_cxx_flags("-nostdlib")
    endif ()
endmacro ()

