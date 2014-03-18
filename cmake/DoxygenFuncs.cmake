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

# Define GEN_DOXYGENT variable and use 
# "embxx_define_doxygen_tgt (<target> <config_file>)" to define
# target for generation of doxygen documentation.

function (embxx_define_doxygen_tgt tgt config_file)
    find_package (Doxygen)
    if (DOXYGEN_FOUND)
        set (match_str "OUTPUT_DIRECTORY[^\n]*")
        set (replacement_str "OUTPUT_DIRECTORY = ${CMAKE_CURRENT_BINARY_DIR}/${tgt}")
        set (output_file "${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf")

        file (READ ${config_file} config_text)
        string (REGEX REPLACE "${match_str}" "${replacement_str}" modified_config_text "${config_text}")
        file (WRITE "${output_file}" "${modified_config_text}")

        add_custom_target (${tgt}
                COMMAND ${DOXYGEN_EXECUTABLE} ${output_file}
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif ()
endfunction ()
