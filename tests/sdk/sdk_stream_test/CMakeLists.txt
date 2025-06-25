cmake_minimum_required(VERSION 2.8)
project(sdk_stream_test)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

if( WIN32 )
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/test-example.vector $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	)

install(PROGRAMS
    $<TARGET_FILE:sdk_stream_test>
    DESTINATION ${CMAKE_INSTALL_BINDIR} )