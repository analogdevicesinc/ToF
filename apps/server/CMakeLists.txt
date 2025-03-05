cmake_minimum_required(VERSION 2.8)
project(aditof-server)

add_executable(${PROJECT_NAME} server.cpp ${PROTO_HDRS} ${PROTO_SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE ${GENERATED_PROTO_FILES_DIR})

# Glog
if(NOT WITH_SUBMODULES AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../glog/.git")
    find_package(glog 0.3.5 REQUIRED)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC glog)
endif()

# Protobuf
if(NOT WITH_SUBMODULES AND NOT EXISTS "${LIBADITOF_SUBMODULE_PATH}/protobuf/.git")
    find_package(Protobuf 3.9.0 REQUIRED)
else()
target_link_libraries(${PROJECT_NAME} PUBLIC libprotobuf)
endif()

# Websockets
if(NOT WITH_SUBMODULES AND NOT EXISTS "${LIBADITOF_SUBMODULE_PATH}/libwebsockets/.git")
        find_package(Libwebsockets REQUIRED)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC websockets_shared)
endif()

# Generating protobuf files
set(NET_PROTO_HRDS "${CMAKE_BINARY_DIR}/buffer.pb.h")
    set(NET_PROTO_SRCS "${CMAKE_BINARY_DIR}/buffer.pb.cc")
    add_custom_command(OUTPUT ${NET_PROTO_HRDS} ${NET_PROTO_SRCS}
                        COMMAND protoc buffer.proto -I ${LIBADITOF_SUBMODULE_PATH}/sdk/src/connections/network --cpp_out=.
                        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    target_sources(${PROJECT_NAME} PRIVATE ${NET_PROTO_HRDS} ${NET_PROTO_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE
            ${CMAKE_BINARY_DIR}
    )
target_link_libraries(${PROJECT_NAME} PRIVATE aditof)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR}/libaditof/libwebsockets/include)

if (USE_ZMQ) 
    message("Included ZMQ libraries")
    set(CPPZMQ_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../libaditof/cppzmq")
    target_include_directories(${PROJECT_NAME} PRIVATE ${CPPZMQ_INSTALL_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE libzmq-static)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
