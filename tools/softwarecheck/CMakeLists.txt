cmake_minimum_required(VERSION 3.10)
project(softwarecheck)

add_executable(${PROJECT_NAME} softwarecheck.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

set(DEPENDENCIES_DIR ../../dependencies/third-party/)
set(SDK_DIR ../../sdk/include)

include_directories(
    ${DEPENDENCIES_DIR}
    ${SDK_DIR}
)

set(SOURCES
${DEPENDENCIES_DIR}/c_json/cJSON.c
)

add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD COMMAND 
	COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBTOFI_LIBDIR_PATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/sw_versions.info" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof cjson)
