cmake_minimum_required(VERSION 3.10)
project(software_check)

add_executable(${PROJECT_NAME} software_check.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

set(DEPENDENCIES_DIR ../../dependencies/third-party/)
set(SDK_DIR ../../libaditof/sdk/include)

include_directories(
    ${DEPENDENCIES_DIR}
    ${SDK_DIR}
)


add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/sw_versions.info" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
)


target_link_libraries(${PROJECT_NAME} PRIVATE aditof cjson)