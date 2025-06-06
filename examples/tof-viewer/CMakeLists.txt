# Copyright (c) Analog Devices, Inc. All rights reserved.
# Minimum Version determined by the following dev environments
# Visual Studio 2017 15.7 - CMake 3.11
cmake_minimum_required(VERSION 3.10)

# set the project name
project(ADIToFGUI)

option(ENBABLE_PASSIVE_IR "Allow passive IR (PCM) mode" OFF)

configure_file(ADIToFConfig.h.in ADIToFConfig.h)

# adding submodules and subdirectories
if( WIN32 )
    set(OS_SPECIFIC_DIR windows)
elseif ( APPLE )
    set(CMAKE_CXX_FLAGS "-x objective-c")
    set(OS_SPECIFIC_SOURCES ${ADIToF_SOURCE_DIR}/fileDialog.mm) 	
elseif ( UNIX )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    set(OS_SPECIFIC_DIR linux)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

set(IMGUI_SOURCE_DIR ${CMAKE_SOURCE_DIR}/dependencies/third-party/imgui)
set(ADIToF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(ADIToFGUI
			   ${ADIToF_SOURCE_DIR}/wmain.cpp
			   ${ADIToF_SOURCE_DIR}/ADIView.cpp
			   ${ADIToF_SOURCE_DIR}/ADIToFRecorder.cpp
			   ${ADIToF_SOURCE_DIR}/ADIToF.cpp
			   ${ADIToF_SOURCE_DIR}/ADIMainWindow.cpp
			   ${ADIToF_SOURCE_DIR}/ADIImGUIExtensions.cpp
			   ${ADIToF_SOURCE_DIR}/ADIController.cpp
               ${ADIToF_SOURCE_DIR}/ADIOpenFile.cpp			   
               ${OS_SPECIFIC_SOURCES}			   
			   )

# Link Libraries
if(UNIX)
	set(OpenGL_GL_PREFERENCE "LEGACY")
endif()

find_package(OpenGL REQUIRED)
if( WIN32 )																	 
	target_link_libraries(ADIToFGUI PRIVATE "${IMGUI_SOURCE_DIR}/src/examples/libs/glfw/lib-vc2010-64/glfw3.lib")
elseif(APPLE)
    target_link_libraries(ADIToFGUI PRIVATE "${IMGUI_SOURCE_DIR}/lib/macOS/glfw/lib-x86_64/libglfw3.a")
elseif (UNIX)
	find_package(glfw3 REQUIRED)
	target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
endif()

target_link_libraries(ADIToFGUI PRIVATE
					  aditof
					  imgui::imgui
					  ${OPENGL_gl_LIBRARY}
					  ${CMAKE_DL_LIBS}
					  cjson
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
						"${IMGUI_SOURCE_DIR}/src/examples/libs/gl3w"
						"${IMGUI_SOURCE_DIR}/src/examples/libs/glfw/include"
						"${IMGUI_SOURCE_DIR}/src/examples"
						"${IMGUI_SOURCE_DIR}/src"
						"${PROJECT_SOURCE_DIR}/src"
				)

target_include_directories(ADIToFGUI PUBLIC
                        "${CMAKE_SOURCE_DIR}/libaditof/sdk/src/cameras/itof-camera")

if (ENBABLE_PASSIVE_IR)
	message(STATUS "Building with Passive IR Mode enabled")
	add_definitions(-DENBABLE_PASSIVE_IR)
endif()

# Copying necessary dlls from SDK and the necessary config files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/tof-tools.config $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIBADITOF_SUBMODULE_PATH}/sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3100.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIBADITOF_SUBMODULE_PATH}/sdk/src/cameras/itof-camera/config/config_adsd3500_adsd3030.json $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    COMMENT "Copying cfgs and libs to build examples for tof-viewer."
	)

if(WIN32)
	# copy sdk lib and dc dll to exe directory
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_LINKER_FILE_DIR:aditof> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
					)
endif()
