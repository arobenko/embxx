# Define GEN_DOXYGENT variable and use 
# "ar_define_doxygen_tgt (<target> <config_file>)" to define
# target for generation of doxygen documentation.

function (ar_define_doxygen_tgt tgt config_file)
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
