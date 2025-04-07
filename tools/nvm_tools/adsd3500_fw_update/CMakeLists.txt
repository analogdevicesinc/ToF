cmake_minimum_required(VERSION 3.10)
project(adsd3500_fw_update)

set(NVM_COMMON ../common)

include_directories(
    ${NVM_COMMON}/include
    .
)

set(SOURCES
    ${NVM_COMMON}/compute_crc.c
    ${NVM_COMMON}/crc_table.c
)

add_executable(${PROJECT_NAME} main.cpp Adsd3500.cpp ${SOURCES})
