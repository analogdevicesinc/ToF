cmake_minimum_required(VERSION 3.10)
project(fw_upgrade)

set(NVM_COMMON ../common)

include_directories(
    ${NVM_COMMON}
)

set(SOURCES
${NVM_COMMON}/nvm_tools_common.cpp
)

add_executable(${PROJECT_NAME} fw_upgrade.cpp ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE crc32)
