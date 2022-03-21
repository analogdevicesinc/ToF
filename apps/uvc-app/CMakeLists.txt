cmake_minimum_required(VERSION 3.0)
project(uvc-app)
add_subdirectory(lib)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -Werror")

include_directories(${CMAKE_BINARY_DIR}/generated)

# Generate config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in
	       ${CMAKE_BINARY_DIR}/generated/config.h)

find_package(Protobuf 3.9.0 REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS buffer.proto)

get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

message(STATUS "foo include dir: ${GENERATED_PROTO_FILES_DIR}")

add_executable(${PROJECT_NAME} uvc-app.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(${PROJECT_NAME} PRIVATE ${Protobuf_INCLUDE_DIRS} ${GENERATED_PROTO_FILES_DIR} )
target_link_libraries(${PROJECT_NAME} PRIVATE uvcgadget aditof ${Protobuf_LIBRARIES} )
target_sources(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tof-sdk-interface.cpp)
