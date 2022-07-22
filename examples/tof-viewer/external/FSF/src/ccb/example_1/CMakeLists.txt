cmake_minimum_required(VERSION 3.7)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)  # not necessary, but encouraged

project(ccb_parser VERSION 0.0.1 LANGUAGES C)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)
set(CCB_DIRECTORY ../include/)

set(SRC_FILES 
    ${CCB_DIRECTORY}/TOF_Calibration_Types.h
    ./read_ccb.c
)

add_executable(ccb_parser ${THIRD_PARTY_SRC} ${SRC_FILES})

target_include_directories(ccb_parser PRIVATE ../include/)

# Copy Data files here for being able to run an out-of-source build.
file(COPY 
    ${CMAKE_CURRENT_SOURCE_DIR}/CCB_1909-0240034.ccb 
DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin/)
