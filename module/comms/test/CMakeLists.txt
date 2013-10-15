# In order to run the unittests the following conditions must be true:
#   - find_package (CxxTest) was exectued, CXXTEST_FOUND is defined and has true value.

if (CXXTEST_FOUND)

#################################################################

function (test_message)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/Message.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.Message")

    set (runner "MessageRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_msg_data_layer)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/MsgDataLayer.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.MsgDataLayer")

    set (runner "MsgDataLayerRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_msg_id_layer)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/MsgIdLayer.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.MsgIdLayer")

    set (runner "MsgIdLayerRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_msg_size_layer)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/MsgSizeLayer.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.MsgSizeLayer")

    set (runner "MsgSizeLayerRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_checksum_layer)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/ChecksumLayer.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.ChecksumLayer")

    set (runner "ChecksumLayerRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

function (test_fields)
    set (tests "${CMAKE_CURRENT_SOURCE_DIR}/Fields.th")

    set (extra_sources)

    set (name "${COMPONENT_NAME}.Fields")

    set (runner "FieldsRunner.cpp")
    
    set (link)

    CXXTEST_ADD_TEST (${name} ${runner} ${tests} ${extra_sources})
    
    target_link_libraries (${name} ${link})
    
endfunction ()

#################################################################

include_directories ("${CXXTEST_INCLUDE_DIR}")

ar_add_cxx_flags ("-Wno-overloaded-virtual")

test_message()
test_msg_data_layer()
test_msg_id_layer()
test_msg_size_layer()
test_checksum_layer()
test_fields()

endif ()
