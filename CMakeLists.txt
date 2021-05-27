cmake_minimum_required(VERSION 2.8)
project(adi_tof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 2)
set(ADITOF_VERSION_MINOR 0)
set(ADITOF_VERSION_PATCH 0)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)
option(WITH_OPENCV "Build opencv bindings?" OFF)
option(WITH_MATLAB "Build matlab bindings?" OFF)
option(WITH_OPEN3D "Build open3d bindings?" OFF)
option(WITH_ROS "Build ros bindings?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(WITH_OFFLINE "Build offline interface?" OFF)
option(USE_CHICONY "Use Chicony 006 camera?" OFF)
option(USE_ITOF "Use ITOF camera?" ON)
option(USE_FXTOF1 "Use FXTOF1 camera?" OFF)
option(WITH_TOOLS "Build tools?" OFF)

if (USE_ITOF)
        if (NOT LIBTOFI_LIBDIR_PATH)
                set(LIBTOFI_LIBDIR_PATH "${CMAKE_SOURCE_DIR}/../libs")
        endif()
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
        
        set(MODE3_FILE "mode3_raw.bin")
        set(MODE5_FILE "mode5_raw.bin")
        set(MODE3_URL "swdownloads.analog.com/cse/aditof/resources/itof/frames/${MODE3_FILE}")
        set(MODE5_URL "swdownloads.analog.com/cse/aditof/resources/itof/frames/${MODE5_FILE}")
        set(MODE3_PATH "${RESOURCES_OFFLINE_DIR}/${MODE3_FILE}")
        set(MODE5_PATH "${RESOURCES_OFFLINE_DIR}/${MODE5_FILE}")

        message("Downloading raw frames into ${RESOURCES_OFFLINE_DIR}")
        if (NOT EXISTS ${MODE3_PATH})
                file(DOWNLOAD "${MODE3_URL}" "${MODE3_PATH}")
        endif()
        if (NOT EXISTS ${MODE5_PATH})
                file(DOWNLOAD "${MODE5_URL}" "${MODE5_PATH}")
        endif()
endif()

if (USE_CHICONY)
        add_definitions(-DCHICONY_006)
endif()

if (USE_FXTOF1)
        add_definitions(-DFXTOF1)
endif()

if (USE_ITOF)
        add_definitions(-DITOF)
endif()

set(CONFIG_DIR_NAME "config")
add_definitions(-DCONFIG_DIR_NAME="${CONFIG_DIR_NAME}")
add_definitions(-DRESOURCES="${RESOURCES_DIR}")

add_subdirectory(sdk)
add_subdirectory(apps)

if(WITH_TOOLS)
        if (MSVC)
                add_subdirectory(tools/deps/wingetopt)
        endif(MSVC)
        add_subdirectory(tools/eeprom-tool)
endif()
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
if (WITH_MATLAB)
        add_subdirectory(bindings/matlab)
endif()
if (WITH_OPEN3D)
        add_subdirectory(bindings/open3D)
endif()
if (WITH_ROS)
        add_subdirectory(bindings/ros)
endif()

############################### Install udev rules #######################################
include (${CMAKE_SOURCE_DIR}/cmake/udev-rules-install.cmake)

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
