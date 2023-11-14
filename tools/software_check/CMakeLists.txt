cmake_minimum_required(VERSION 3.10)
project(software_check)

add_executable(${PROJECT_NAME} software_check.cpp ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

set(DEPENDENCIES_DIR ../../dependencies/third-party/)
set(SDK_DIR ../../sdk/include)

include_directories(
    ${DEPENDENCIES_DIR}
    ${SDK_DIR}
)

if (WIN32)
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/sw_versions.info" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
    if (WITH_NETWORK)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${LWS_CMAKE_DIR}/../bin ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
        )
    endif()
    if (WITH_GLOG_DEPENDENCY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${GLOG_BIN_DIR} ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
        )
    endif()
else()
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD COMMAND 
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/sdk/" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/sw_versions.info" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE aditof cjson)