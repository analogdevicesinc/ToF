cmake_minimum_required(VERSION 3.7)
project(ccb_example)

set(CCB_DIRECTORY ../include/)

set(SRC_FILES 
    ../src/ccb.c
    ./example.c
)

add_executable(ccb_example ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CCB_DIRECTORY}
)

target_link_libraries(ccb_example)
