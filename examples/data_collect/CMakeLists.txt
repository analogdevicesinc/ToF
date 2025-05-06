cmake_minimum_required(VERSION 3.10)
project(data_collect)

add_executable(${PROJECT_NAME} main.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

if (NOT WIN32)
    find_library(PTHREAD_LIBRARIES pthread)
    if (PTHREAD_LIBRARIES)
       target_link_libraries(${PROJECT_NAME} PRIVATE ${PTHREAD_LIBRARIES})
    else()
       message(FATAL_ERROR "pthread libraries not found!")
    endif()
endif()

if( WIN32 )
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
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

else()
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
)
endif()

if ( WIN32 )
   if (MSVC)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:4194304")
   else ()
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--stack,4194304")
   endif()

   set_target_properties(data_collect PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$(ProjectDir)\\$(Configuration)")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)
