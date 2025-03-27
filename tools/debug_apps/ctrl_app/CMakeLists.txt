cmake_minimum_required(VERSION 3.10)
project(ctrl_app)

add_executable(${PROJECT_NAME} ctrl_app.cpp)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/infile.txt
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/readme.md
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMENT "Copying infile.txt."
)
