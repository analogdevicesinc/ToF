cmake_minimum_required(VERSION 3.0)
project(adi_tof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 4)
set(ADITOF_VERSION_MINOR 3)
set(ADITOF_VERSION_PATCH 0)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)
option(WITH_OPENCV "Build opencv bindings?" OFF)
option(WITH_OPEN3D "Build open3d bindings?" OFF)
option(WITH_ROS "Build ros bindings?" OFF)
option(WITH_ROS2 "Build ros2 bindings?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(WITH_OFFLINE "Build offline interface?" OFF)
option(USE_DEPTH_COMPUTE_STUBS "Use empty generated libs?" ON)
option(USE_DEPTH_COMPUTE_OPENSOURCE "Use an open source implementation?" OFF)
option(WITH_GLOG_DEPENDENCY "Build with GLOG dependency to be used for logging" ON)
option(WITH_PROTOBUF_DEPENDENCY "Build with PROTOBUF dependency to be used for serializing messages" ON)
option(WITH_COMMAND_LINE_TOOLS "Build with embedded Linux command line tools" ON)
option(CI_BUILD "Build for CI" OFF)


################################## Git ########################################
include(FindGit OPTIONAL)
if (GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_VARIABLE ADITOFSDK_GIT_REPO
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)

	if ("${ADITOFSDK_GIT_REPO}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_COMMIT
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_BRANCH
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	endif()
endif()

add_definitions(-DADITOFSDK_GIT_COMMIT="${ADITOFSDK_GIT_COMMIT}")
add_definitions(-DADITOFSDK_GIT_BRANCH="${ADITOFSDK_GIT_BRANCH}")

########################## Checks and validations #############################

set(DEPTH_COMPUTE_TYPE 0)
set(ALL_DEPTH_COMPUTE_OPTIONS USE_DEPTH_COMPUTE_STUBS;USE_DEPTH_COMPUTE_OPENSOURCE)
foreach(depth_compute IN LISTS ALL_DEPTH_COMPUTE_OPTIONS)
    if(${depth_compute})
        math(EXPR DEPTH_COMPUTE_TYPE "${DEPTH_COMPUTE_TYPE}+1")
    endif()
endforeach()

if(${DEPTH_COMPUTE_TYPE} GREATER 1)
        message(FATAL_ERROR "More than one Depth Compute library source was selected. Cmake will exit.")
endif()

############################## Rest of cmake ##################################

# Check whether we configure things for remote or we're on target
set(SELECTED_PLATFORMS 0)
set(ALL_PLATFORMS NXP;NVIDIA)
foreach(platform IN LISTS ALL_PLATFORMS)
    if(${platform})
        math(EXPR SELECTED_PLATFORMS "${SELECTED_PLATFORMS}+1")
    endif()
endforeach()

set(ON_TARGET FALSE)
if(${SELECTED_PLATFORMS} GREATER 1)
        message(FATAL_ERROR "More than one platform was selected. \nPlease select a single platform. CMake will exit.")
elseif(${SELECTED_PLATFORMS} EQUAL 1)
    add_definitions(-DTARGET)
    set(ON_TARGET TRUE)
    set(USE_DEPTH_COMPUTE_STUBS OFF)
endif()

if(CI_BUILD)
    set(USE_DEPTH_COMPUTE_STUBS ON)
endif()

if (USE_ADSD3030)
        add_definitions(-DADSD3030)
endif()

if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

set(RESOURCES_DIR "${CMAKE_BINARY_DIR}/resources")
make_directory(${RESOURCES_DIR})

if (WITH_OFFLINE)
        add_definitions(-DHAS_OFFLINE)

        set(RESOURCES_OFFLINE_DIR "${RESOURCES_DIR}/offline")
        make_directory(${RESOURCES_OFFLINE_DIR})
        
        set(MODE_FILE "lr-qnative.bin" "sr-qnative.bin")
        message("Downloading raw frames into ${RESOURCES_OFFLINE_DIR}")
        foreach(MODE IN LISTS MODE_FILE)
                set(MODE_URL "swdownloads.analog.com/cse/aditof/resources/itof/frames/${MODE}")
                set(MODE_PATH "${RESOURCES_OFFLINE_DIR}/${MODE}")

                if (NOT EXISTS ${MODE_PATH})
                        file(DOWNLOAD "${MODE_URL}" "${MODE_PATH}")
                endif()
        endforeach()
endif()

if(WITH_GLOG_DEPENDENCY)
        add_definitions(-DUSE_GLOG)
endif()

if(NOT WITH_PROTOBUF_DEPENDENCY)
        if(NOT NXP AND NOT NVIDIA)
                message(FATAL_ERROR "SDK can be built without protobuf only on target builds!")
        endif()
else()
        add_definitions(-DUSE_PROTOBUF)
endif()

if(USE_DEPTH_COMPUTE_STUBS)
	set(LIBTOFI_LIBDIR_PATH "${CMAKE_BINARY_DIR}/sdk/common/adi/depth-compute-stub")
elseif(USE_DEPTH_COMPUTE_OPENSOURCE)
	set(LIBTOFI_LIBDIR_PATH "${CMAKE_BINARY_DIR}/sdk/common/adi/depth-compute-opensource")
else()
        set(LIBTOFI_LIBDIR_PATH "${CMAKE_SOURCE_DIR}/../libs")
endif()

if(WIN32 AND (USE_DEPTH_COMPUTE_STUBS OR USE_DEPTH_COMPUTE_OPENSOURCE))
        set(LIBTOFI_LIBDIR_PATH "${LIBTOFI_LIBDIR_PATH}/${CMAKE_BUILD_TYPE}")
endif()

set(CONFIG_DIR_NAME "config")
add_definitions(-DCONFIG_DIR_NAME="${CONFIG_DIR_NAME}")
add_definitions(-DRESOURCES="${RESOURCES_DIR}")

add_subdirectory(dependencies)
add_subdirectory(apps)
add_subdirectory(sdk)

if (WITH_EXAMPLES)
        add_subdirectory(examples)
endif()
if (WITH_DOC)
        add_subdirectory(doc)
endif()
if (WITH_PYTHON)
        add_subdirectory(bindings/python)
endif()
if (WITH_OPENCV)
        add_subdirectory(bindings/opencv)
endif()
if (WITH_OPEN3D)
        add_subdirectory(bindings/open3D)
endif()
if (WITH_ROS)
        add_subdirectory(bindings/ros)
endif()
if (WITH_ROS2)
        add_subdirectory(bindings/ros2)
endif()
if (WITH_COMMAND_LINE_TOOLS)
        add_subdirectory(tools)
endif()

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
