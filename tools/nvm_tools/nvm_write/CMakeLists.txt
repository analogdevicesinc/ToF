cmake_minimum_required(VERSION 3.10)
project(nvm_write)

set(CRC_LIB ../../../sdk/src/cameras/itof-camera)
set(NVM_COMMON ../common)

include_directories(
    ${CRC_LIB}
    ${NVM_COMMON}
)

set(SOURCES
${CRC_LIB}/crc.c
${NVM_COMMON}/nvm_tools_common.cpp
)

add_executable(${PROJECT_NAME} nvm_write.cpp ${SOURCES})
