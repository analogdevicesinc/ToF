cmake_minimum_required(VERSION 3.0)
project(cjson)

add_library(${PROJECT_NAME} STATIC
    cJSON.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)