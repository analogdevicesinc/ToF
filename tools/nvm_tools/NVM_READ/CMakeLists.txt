cmake_minimum_required(VERSION 3.10)
project(NVM_READ)

set(NVM_COMMON ../common)

include_directories(
    ${NVM_COMMON}
)

set(SOURCES
    ${NVM_COMMON}/compute_crc.c
    ${NVM_COMMON}/crc_table.c
)

add_executable(${PROJECT_NAME} NVM_READ.cpp ${SOURCES})
