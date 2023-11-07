cmake_minimum_required(VERSION 2.8)
project(aditof-http-server)

add_definitions(-DHTTP_LOCAL_RESOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Libwebsockets REQUIRED)

add_executable(${PROJECT_NAME} test-server.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${Protobuf_LIBRARIES} ${LIBWEBSOCKETS_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${Protobuf_INCLUDE_DIRS} ${GENERATED_PROTO_FILES_DIR} ${LIBWEBSOCKETS_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

# Copying config files and depth compute libraries from sdk/config, sdk/lib folders to build/examples/first-frame and build/first-frame/Debug or Release folders    
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/${CONFIG_DIR_NAME}/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3100.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3030.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMENT "Copying cfgs and libs to apps for http-server."
)