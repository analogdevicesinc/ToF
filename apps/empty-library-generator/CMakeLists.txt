cmake_minimum_required(VERSION 2.8)
project(empty-library-generator)
include(GNUInstallDirs)

set (LIB_SOURCES "${CMAKE_SOURCE_DIR}/sdk/src/cameras/itof-camera/tofi/")

add_library(tofi_compute SHARED ${LIB_SOURCES}/tofiComputeDefault.cpp)
set_target_properties(tofi_compute PROPERTIES
    PUBLIC_HEADER ${LIB_SOURCES}/tofi_compute.h)

add_library(tofi_config SHARED ${LIB_SOURCES}/tofiConfigDefault.cpp)
set_target_properties(tofi_config PROPERTIES
    PUBLIC_HEADER ${LIB_SOURCES}/tofi_config.h)

install(TARGETS tofi_compute)
install(TARGETS tofi_config)
