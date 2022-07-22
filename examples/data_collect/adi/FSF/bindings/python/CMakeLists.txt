cmake_minimum_required(VERSION 3.10)

find_package(pybind11 REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(../../include)

pybind11_add_module(fsf_python fsf_python.cpp)

target_link_libraries(fsf_python PRIVATE fsf_common fsf)
