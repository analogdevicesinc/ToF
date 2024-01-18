cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # not necessary, but encouraged

project(point_cloud VERSION 0.0.1 LANGUAGES C CXX)


if (MSVC)
    # to avoid warning from using unsafe/deprecated funtions
    # in windows, like from using fopen() instead of fopen_s()
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)


set(SRC_DIR ${CMAKE_SOURCE_DIR}/examples/point_cloud)
set(TOFI_CCB ${CMAKE_SOURCE_DIR}/sdk/common/adi/ccb)

set(THIRD_PARTY_SRC 
    ${TOFI_CCB}/src/ccb.c
    ${TOFI_CCB}/include/ccb.h
)

set(SRC_FILES 
		${SRC_DIR}/main.cpp
        ${SRC_DIR}/point_cloud.cpp
        ${SRC_DIR}/opencv_undistort.cpp
        ${SRC_DIR}/point_cloud.h
   )
    
add_executable(point_cloud ${THIRD_PARTY_SRC} ${SRC_FILES})

target_include_directories(point_cloud PRIVATE
	    ${TOFI_CCB}/include/
        ${SRC_DIR}/
)

if( WIN32 )
    if (WITH_GLOG_DEPENDENCY)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOG_BIN_DIR} ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
    )
    endif()
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser)
