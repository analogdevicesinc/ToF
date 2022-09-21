cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # not necessary, but encouraged

project(point_cloud VERSION 0.0.1 LANGUAGES C CXX)


if (MSVC)
    # to avoid warning from using unsafe/deprecated funtions
    # in windows, like from using fopen() instead of fopen_s()
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)



set(SRC_DIR ./)
set(DOCOPT_SRC ../data_collect/third_party/doc_opt/)
set(TOFI_CCB ../../sdk/common/adi/ccb)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY $<CONFIG>/)

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
        ${DOCOPT_SRC}
)

