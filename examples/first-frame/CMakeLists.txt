cmake_minimum_required(VERSION 3.0)
project(first-frame)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)

if( WIN32 )
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
endif()
