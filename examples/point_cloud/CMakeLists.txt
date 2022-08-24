cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # not necessary, but encouraged

project(point_cloud VERSION 0.0.1 LANGUAGES C CXX)


if (MSVC)
    # to avoid warning from using unsafe/deprecated funtions
    # in windows, like from using fopen() instead of fopen_s()
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)



set(SRC_DIR point_cloud)
set(DOCOPT_SRC ./third_party/doc_opt/)
set(TOFI_CCB ./ccb/)
set(CMAKE_CURR_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

set(THIRD_PARTY_SRC 
	${DOCOPT_SRC}/docopt.cpp
	${DOCOPT_SRC}/docopt.h
	${DOCOPT_SRC}/docopt_value.h
	${DOCOPT_SRC}/docopt_util.h
	${DOCOPT_SRC}/docopt_private.h
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
        ${DOCOPT_SRC}/
        ${DEPTH_COMPUTE_COMMON}
)

file(MAKE_DIRECTORY ${CMAKE_CURR_BUILD_DIR}/bin/)
