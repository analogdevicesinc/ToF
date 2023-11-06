cmake_minimum_required(VERSION 3.0)
project(dependecies)

add_subdirectory(adi/command_parser)
add_subdirectory(third-party/c_json)
add_subdirectory(third-party/crc32)
if (WITH_EXAMPLES AND NOT NXP)
        add_subdirectory(third-party/imgui)
endif()