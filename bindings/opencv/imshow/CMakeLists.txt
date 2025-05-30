cmake_minimum_required(VERSION 3.0)

project(aditof-opencv-imshow)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${OpenCV_LIBS})

# Copying config files and depth compute libraries
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../../libaditof/sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/${CONFIG_DIR_NAME}/.
COMMENT "Copying cfgs and libs to build examples "
)

if( WIN32 )
    # Copying the sdk binary
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/libaditof/sdk/$<CONFIGURATION>" $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
    )
endif()
