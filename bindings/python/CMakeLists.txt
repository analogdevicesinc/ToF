cmake_minimum_required(VERSION 3.0)

function(PopulateforPython targetdir)
    add_subdirectory(${targetdir})
    
    file(GLOB PythonSources "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/*.py")
    
    if(WIN32)
        add_custom_command( TARGET aditofpython POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/$<CONFIG>" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
            COMMAND ${CMAKE_COMMAND} -E copy ${PythonSources} "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/${TARGET_OUTPUT_NAME}" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/$<CONFIG>"
        )
    elseif(UNIX)
        add_custom_command( TARGET aditofpython POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
            COMMAND ${CMAKE_COMMAND} -E copy ${PythonSources} "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_OUTPUT_NAME}" "${CMAKE_CURRENT_BINARY_DIR}/${targetdir}/"
        )
    endif()


endfunction()

add_subdirectory(pybind11)

include_directories(${LIBADITOF_SUBMODULE_PATH}/sdk/includes)

pybind11_add_module(aditofpython aditofpython.cpp)

target_link_libraries(aditofpython PRIVATE aditof)

get_target_property(target_prefix aditofpython PREFIX)
get_target_property(target_suffix aditofpython SUFFIX)
set(TARGET_OUTPUT_NAME "${target_prefix}aditofpython${target_suffix}")

if (WITH_EXAMPLES)
    PopulateforPython("examples/dnn")
    PopulateforPython("examples/first_frame")
    PopulateforPython("examples/gesture_rec")
    PopulateforPython("examples/streaming")
    PopulateforPython("examples/skeletal_tracking")
    PopulateforPython("examples/skeletal_tracking_in_pointcloud")
    PopulateforPython("examples/saveCCBToFile")
    PopulateforPython("examples/data_collect")
    PopulateforPython("examples/showPointCloud")
    PopulateforPython("examples/dual_cameras")

endif()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/setup.py.cmakein
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

if(NOT WIN32)
        install(CODE "execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_EXECUTABLE} setup.py install --prefix=${CMAKE_INSTALL_PREFIX})")
endif()
