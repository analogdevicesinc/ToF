cmake_minimum_required(VERSION 3.0)
project(crc32)

add_library(${PROJECT_NAME} STATIC
    crc.c
)

if (NOT WIN32) 
    set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)
