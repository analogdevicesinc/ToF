# Copyright (c) Analog Devices, Inc. All rights reserved.
# Minimum Version determined by the following dev environments
# Visual Studio 2017 15.7 - CMake 3.11
cmake_minimum_required(VERSION 3.10)

# set the project name
project(ADIToFGUI)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(ADIToFConfig.h.in ADIToFConfig.h)

# adding submodules and subdirectories
add_subdirectory(external/imgui)
add_subdirectory(external/libusb)
add_subdirectory(src/usbcommand/ADIUSBcmd)
add_subdirectory(external/FSF)
if( WIN32 )
    set(OS_SPECIFIC_DIR windows)
elseif ( UNIX )
    set(OS_SPECIFIC_DIR linux)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

set(ADIToF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(ADIToFGUI
			   ${ADIToF_SOURCE_DIR}/wmain.cpp
			   ${ADIToF_SOURCE_DIR}/ADIView.cpp
			   ${ADIToF_SOURCE_DIR}/ADIToFRecorder.cpp
			   ${ADIToF_SOURCE_DIR}/ADIToF.cpp
			   ${ADIToF_SOURCE_DIR}/ADIMainWindow.cpp
			   ${ADIToF_SOURCE_DIR}/ADIInformationPane.cpp
			   ${ADIToF_SOURCE_DIR}/ADIImGUIExtensions.cpp
			   ${ADIToF_SOURCE_DIR}/ADIController.cpp
			   ${ADIToF_SOURCE_DIR}/ADIBinToFSF.cpp
			   )

# Link Libraries
find_package(OpenGL REQUIRED)																		 
target_link_libraries(ADIToFGUI PRIVATE "${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/glfw/lib-vc2010-64/glfw3.lib")
target_link_libraries(ADIToFGUI PRIVATE ${OPENGL_gl_LIBRARY})
target_link_libraries(ADIToFGUI PRIVATE
					  imgui::imgui					  
					  ADIUSBcmd
					  LibUSB::LibUSB
					  aditof
					  fsf
					  fsf_common
					 )
set_target_properties(ADIToFGUI PROPERTIES
LINK_FLAGS /SUBSYSTEM:WINDOWS
)

target_include_directories(ADIToFGUI PUBLIC
				"${PROJECT_BINARY_DIR}"
				"${PROJECT_SOURCE_DIR}/include"
                                "${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/gl3w"
                                "${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/glfw/include"
                                "${PROJECT_SOURCE_DIR}/external/imgui/src/examples"
                                "${PROJECT_SOURCE_DIR}/external/imgui/src"
				"${PROJECT_SOURCE_DIR}/src/usbcommand/ADIUSBcmd"
				"${PROJECT_SOURCE_DIR}/src"
                                "${PROJECT_SOURCE_DIR}/external/libusb/src/libusb"
                                "${PROJECT_SOURCE_DIR}/external/FSF/src/FSF/src"
                                "${PROJECT_SOURCE_DIR}/external/FSF/src/FSF/include"
				)
# Still needed?
target_include_directories(ADIToFGUI PUBLIC
                        "${CMAKE_SOURCE_DIR}/sdk/src/cameras/itof-camera")

#target_include_directories(ADIToFGUI PUBLIC
#                        "${PROJECT_SOURCE_DIR}/external/aditof_sdk/src/sdk/include"
#                        "${PROJECT_SOURCE_DIR}/external/aditof_sdk/src/sdk/include/aditof"
#                        "${PROJECT_SOURCE_DIR}/external/aditof_sdk/src/examples/aditof-demo")
#endif()

# Copying necessary dlls from SDK and the necessary config files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBTOFI_LIBDIR_PATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/../
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/tof-viewer_config.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/tof-viewer_config.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/..
	)

if(WIN32)
	# copy sdk lib to exe directory
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_LINKER_FILE_DIR:aditof> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
					VERBATIM)
endif()
