cmake_minimum_required(VERSION 3.0)

function(PopulateforPython targetdir)
    add_subdirectory(${targetdir})
    
    file(GLOB PythonSources "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/*.py")
    
    if(WIN32)
        add_custom_command( TARGET aditofpython POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/$<CONFIG>" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>/config"
            COMMAND ${CMAKE_COMMAND} -E copy ${PythonSources} "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/${TARGET_OUTPUT_NAME}" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
        )
    elseif(UNIX)
        add_custom_command( TARGET aditofpython POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/config"
            COMMAND ${CMAKE_COMMAND} -E copy ${PythonSources} "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
        )
    endif()
    if (WIN32)
        if (EXISTS "${GLOG_BIN_DIR}")
            if (WITH_GLOG_DEPENDENCY)
                add_custom_command( TARGET aditofpython POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOG_BIN_DIR} "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>")
            endif()
        endif()

        if (EXISTS "${LWS_CMAKE_DIR}/../bin")
            if (WITH_NETWORK)
                add_custom_command(TARGET aditofpython POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${LWS_CMAKE_DIR}/../bin "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>")
            endif()
        endif()
    
    endif()

endfunction()

find_package(pybind11 REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(../../sdk/includes)

pybind11_add_module(aditofpython aditofpython.cpp)

target_link_libraries(aditofpython PRIVATE aditof)

get_target_property(target_prefix aditofpython PREFIX)
get_target_property(target_suffix aditofpython SUFFIX)
set(TARGET_OUTPUT_NAME "${target_prefix}aditofpython${target_suffix}")

if (WITH_EXAMPLES)
    PopulateforPython("examples/dnn")
    PopulateforPython("examples/first_frame")
    PopulateforPython("examples/streaming")
    PopulateforPython("examples/skeletal_tracking")
    PopulateforPython("examples/saveCCBToFile")

endif()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/setup.py.cmakein
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

if(NOT WIN32)
        install(CODE "execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_EXECUTABLE} setup.py install --prefix=${CMAKE_INSTALL_PREFIX})")
endif()
