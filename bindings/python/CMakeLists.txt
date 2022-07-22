cmake_minimum_required(VERSION 3.0)

find_package(pybind11 REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(../../sdk/includes)

pybind11_add_module(aditofpython aditofpython.cpp)

target_link_libraries(aditofpython PRIVATE aditof)

get_target_property(target_prefix aditofpython PREFIX)
get_target_property(target_suffix aditofpython SUFFIX)
set(TARGET_OUTPUT_NAME "${target_prefix}aditofpython${target_suffix}")

add_subdirectory(examples/dnn)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/setup.py.cmakein
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

if(NOT WIN32)
        install(CODE "execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_EXECUTABLE} setup.py install --prefix=${CMAKE_INSTALL_PREFIX})")
endif()