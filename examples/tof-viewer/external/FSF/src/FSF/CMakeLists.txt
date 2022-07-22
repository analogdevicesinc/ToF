cmake_minimum_required(VERSION 3.10)
project(fsf_file_parser_utility)

set(CXX_STANDARD 11)
set(CXX_STANDARD ON)

if (CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W3 /WX")
endif()

############################### Options #######################################
option(WITH_EXAMPLES "Build example?" ON)
option(WITH_PYTHON "Build python bindings?" OFF)

add_subdirectory(src)

if (WITH_EXAMPLES)
    add_subdirectory(example)
endif()

if (WITH_EXAMPLES)
    add_subdirectory(example_c)
endif()

if (WITH_PYTHON)
    add_subdirectory(bindings/python)
    configure_file(${CMAKE_SOURCE_DIR}/bindings/python/fsf_example.py ./bindings/python/fsf_example.py COPYONLY)
endif()
