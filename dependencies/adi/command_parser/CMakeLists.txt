cmake_minimum_required(VERSION 3.0)
project(command_parser)

add_library(${PROJECT_NAME} STATIC
    command_parser.cpp
)

if(WITH_GLOG_DEPENDENCY)
    find_package(glog 0.3.5 REQUIRED)
    set(GLOG_BIN_DIR "${glog_DIR}/../../../bin")
    set(GLOG_BIN_DIR ${GLOG_BIN_DIR} PARENT_SCOPE)
endif()

if (WITH_GLOG_DEPENDENCY)
    target_link_libraries( ${PROJECT_NAME}
    PUBLIC
        glog::glog
    )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    PRIVATE
        .
)