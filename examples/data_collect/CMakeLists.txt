cmake_minimum_required(VERSION 3.10)
project(data_collect)

add_executable(${PROJECT_NAME} main.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

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
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
endif()

if ( WIN32 )
   if (MSVC)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:4194304")
   else ()
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--stack,4194304")
   endif()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)
