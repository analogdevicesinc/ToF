cmake_minimum_required(VERSION 3.0)

project(PointCloud-Open3D)

message(STATUS "Open3D ${Open3D_VERSION}")

# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} main.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof command_parser ${Open3D_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})

# Copying necessary dlls from SDK and the necessary config files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/libaditof/sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/${CONFIG_DIR_NAME}/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/libaditof/sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3100.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/libaditof/sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3030.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMENT "Copying cfgs and libs to build examples for tof-viewer."
	)

if(WIN32)
	# copy sdk lib and dc dll to exe directory
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_LINKER_FILE_DIR:aditof> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
					VERBATIM
					COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/libaditof/sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/../${CONFIG_DIR_NAME}/.
					)
	if (WITH_NETWORK)
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${ZeroMQ_BINARY_DIR}/bin/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
		)
	endif()
	if (WITH_GLOG_DEPENDENCY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${glog_BINARY_DIR}/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
        )
    endif()
	if (WITH_PROTOBUF_DEPENDENCY)
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${protobuf_BINARY_DIR}/$<CONFIG> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
		)
	endif()
endif()

