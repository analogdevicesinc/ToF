cmake_minimum_required(VERSION 3.0)
project(first-frame)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)

if( WIN32 )
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
    if (WITH_NETWORK)
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${libwebsockets_BINARY_DIR}/bin/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
		)
	endif()
    if (WITH_GLOG_DEPENDENCY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${glog_BINARY_DIR}/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
        )
    endif()
    if (WITH_PROTOBUF_DEPENDENCY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${protobuf_BINARY_DIR}/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
        )
    endif()
endif()
