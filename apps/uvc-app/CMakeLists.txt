cmake_minimum_required(VERSION 3.0)
project(uvc-app)
add_subdirectory(lib)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -Werror")

include_directories(${CMAKE_BINARY_DIR}/generated)

# Generate config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in
	       ${CMAKE_BINARY_DIR}/generated/config.h)

add_executable(uvc-app uvc-app.cpp)

target_include_directories(uvc-app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${Protobuf_INCLUDE_DIRS} ${GENERATED_PROTO_FILES_DIR} )
target_link_libraries(uvc-app PRIVATE uvcgadget aditof)
