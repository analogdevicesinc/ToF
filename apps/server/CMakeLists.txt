cmake_minimum_required(VERSION 2.8)
project(aditof-server)

find_package(Protobuf 3.9.0 REQUIRED)
find_package(Libwebsockets REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS buffer.proto)

get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

add_executable(${PROJECT_NAME} server.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${Protobuf_LIBRARIES} ${LIBWEBSOCKETS_LIBRARIES})

if (NOT USE_DEPTH_COMPUTE_STUBS)
	add_library(libtofi_compute SHARED IMPORTED)
	set_target_properties(libtofi_compute PROPERTIES
 	IMPORTED_LOCATION "${LIBTOFI_LIBDIR_PATH}/libtofi_compute.so"
    	IMPORTED_NO_SONAME 1
	)

   	add_library(libtofi_config SHARED IMPORTED)
   	set_target_properties(libtofi_config PROPERTIES
    	IMPORTED_LOCATION "${LIBTOFI_LIBDIR_PATH}/libtofi_config.so"
    	IMPORTED_NO_SONAME 1
	)

    target_link_libraries(${PROJECT_NAME} PRIVATE libtofi_compute)
    target_link_libraries(${PROJECT_NAME} PRIVATE libtofi_config)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${Protobuf_INCLUDE_DIRS} ${GENERATED_PROTO_FILES_DIR} ${LIBWEBSOCKETS_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
