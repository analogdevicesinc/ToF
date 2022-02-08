# Copyright (c) Analog Devices, Inc. All rights reserved.
# Minimum Version determined by the following dev environments
# Visual Studio 2017 15.7 - CMake 3.11
cmake_minimum_required(VERSION 3.10)

# set the project name
project(ADIToFGUI)

option(ENBABLE_PASSIVE_IR "Allow passive IR (PCM) mode" OFF)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(ADIToFConfig.h.in ADIToFConfig.h)

# adding submodules and subdirectories
add_subdirectory(external/imgui)
add_subdirectory(external/FSF)
if( WIN32 )
    set(OS_SPECIFIC_DIR windows)
elseif ( APPLE )
    set(CMAKE_CXX_FLAGS "-x objective-c")
    set(OS_SPECIFIC_SOURCES ${ADIToF_SOURCE_DIR}/fileDialog.mm) 	
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
               ${ADIToF_SOURCE_DIR}/ADIOpenFile.cpp			   
               ${OS_SPECIFIC_SOURCES}			   
			   )

# Link Libraries
find_package(OpenGL REQUIRED)
if( WIN32 )																	 
	target_link_libraries(ADIToFGUI PRIVATE "${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/glfw/lib-vc2010-64/glfw3.lib")
elseif(APPLE)
    target_link_libraries(ADIToFGUI PRIVATE "${PROJECT_SOURCE_DIR}/external/imgui/lib/macOS/glfw/lib-x86_64/libglfw3.a")
elseif (UNIX)
	find_package(glfw3 REQUIRED)
	target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
endif()

target_link_libraries(ADIToFGUI PRIVATE
					  aditof
					  imgui::imgui
					  ${OPENGL_gl_LIBRARY}
					  ${CMAKE_DL_LIBS}
					  fsf_common
					  fsf
					 )

if( WIN32 )					 
	set_target_properties(ADIToFGUI PROPERTIES
	LINK_FLAGS /SUBSYSTEM:WINDOWS
	)
elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-framework Carbon -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo")
endif()
	
target_include_directories(ADIToFGUI PUBLIC
						"${PROJECT_BINARY_DIR}"
						"${PROJECT_SOURCE_DIR}/include"
						"${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/gl3w"
						"${PROJECT_SOURCE_DIR}/external/imgui/src/examples/libs/glfw/include"
						"${PROJECT_SOURCE_DIR}/external/imgui/src/examples"
						"${PROJECT_SOURCE_DIR}/external/imgui/src"
						"${PROJECT_SOURCE_DIR}/src"
						"${PROJECT_SOURCE_DIR}/external/FSF/src/FSF/src"
						"${PROJECT_SOURCE_DIR}/external/FSF/src/FSF/include"
				)

target_include_directories(ADIToFGUI PUBLIC
                        "${CMAKE_SOURCE_DIR}/sdk/src/cameras/itof-camera")

if (ENBABLE_PASSIVE_IR)
	message(STATUS "Building with Passive IR Mode enabled")
	add_definitions(-DENBABLE_PASSIVE_IR)
endif()

# Copying necessary dlls from SDK and the necessary config files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBTOFI_LIBDIR_PATH} $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/${CONFIG_DIR_NAME}/.
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/tof-viewer_config.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/tof-viewer_config.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/..
	)

if(WIN32)
	# copy sdk lib to exe directory
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_LINKER_FILE_DIR:aditof> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
					VERBATIM
					COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/../${CONFIG_DIR_NAME}/.
					)
endif()
