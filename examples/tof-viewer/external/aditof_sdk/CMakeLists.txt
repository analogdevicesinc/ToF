#add_definitions(-DSDK_EXPORTS)
#find_package(glog 0.3.5 REQUIRED)
#find_package(Protobuf 3.9.0 REQUIRED)
#find_package(Libwebsockets REQUIRED)
#protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS src/buffer.proto)
#get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

# Create build folder for aditof SDK
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/build)

# Compile this SDK first
execute_process(COMMAND cmake -DCMAKE_PREFIX_PATH=${PROJECT_SOURCE_DIR}/external/protobuf/src/build_3_9_0/local_path/protobuf -G "Visual Studio 14 2015 Win64" -DWITH_EXAMPLES=off ..
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/build
				)

execute_process(COMMAND cmake --build . --config Release
				WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/build
				)

INCLUDE_EXTERNAL_MSPROJECT(aditof ${CMAKE_CURRENT_SOURCE_DIR}/src/build/sdk/aditof.vcxproj)

##set(ADITOF_PATH ${PROJECT_SOURCE_DIR}/external/aditof_sdk/src/build/sdk
#add_library(aditof
#			STATIC
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/build/sdk/buffer.pb.cc"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/calibration.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/camera_96tof1.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/camera_96tof1_specifics.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/camera_chicony.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/camera_chicony_specifics.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/camera_factory.cpp"
#			#"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/device_enumerator_ethernet.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/device_enumerator_factory.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/device_enumerator_impl.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/device_factory.cpp"
#			#"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/ethernet_device.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/filters_factory.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/frame.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/frame_impl.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/frame_operations.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/local_device.cpp"
#			#"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/network.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/system.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/system_impl.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/usb_device.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/utils.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/variance_filter.cpp"
#			)
#			
#			
#target_include_directories(aditof PUBLIC 
#							${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/windows
#							${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src
#							${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/include
#							${CMAKE_CURRENT_SOURCE_DIR}/src/build/sdk
#							)			
#add_library(aditof::aditof ALIAS aditof)
#
#if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
#    target_sources(aditof PRIVATE
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/windows/device_enumerator_windows.cpp"
#			"${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/src/windows/usb_device_windows.cpp"
#			)
#	target_include_directories(aditof PUBLIC 
#							"${PROJECT_SOURCE_DIR}/external/glog/src/build_0_3_5/local_path/glog/include"
#							"${PROJECT_SOURCE_DIR}/external/protobuf/src/build_3_9_0/local_path/protobuf/include"
#							)
#endif()