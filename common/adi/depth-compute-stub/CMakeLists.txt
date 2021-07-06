cmake_minimum_required(VERSION 2.8)
project(depth-compute-stub)
include(GNUInstallDirs)

set (LIB_SOURCES "${CMAKE_SOURCE_DIR}/sdk/src/cameras/itof-camera/tofi/")

add_library(tofi_compute SHARED ${LIB_SOURCES}/tofiCompute.cpp)
set_target_properties(tofi_compute PROPERTIES
    PUBLIC_HEADER ${LIB_SOURCES}/tofi_compute.h)

add_library(tofi_config SHARED ${LIB_SOURCES}/tofiConfig.cpp)
set_target_properties(tofi_config PROPERTIES
    PUBLIC_HEADER ${LIB_SOURCES}/tofi_config.h)

add_custom_command(TARGET ${PROJECT-NAME} PRE-BUILD
                    COMMAND ${CMAKE_COMMAND} -E --build ./ --target install
)

install(TARGETS tofi_compute)
install(TARGETS tofi_config)
