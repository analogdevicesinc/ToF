cmake_minimum_required(VERSION 2.8)
project(depth-compute-stub)

set (LIB_HEADERS "${CMAKE_SOURCE_DIR}/sdk/src/cameras/itof-camera/tofi/")
set (CCB_HEADERS "${CMAKE_SOURCE_DIR}/sdk/common/adi/ccb/include")

add_library(tofi_compute SHARED src/tofiCompute.cpp)
target_include_directories(tofi_compute
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_HEADERS}>
    PRIVATE
        src/
)

add_library(tofi_config SHARED src/tofiConfig.cpp)
target_include_directories(tofi_config
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_HEADERS}>
        $<BUILD_INTERFACE:${CCB_HEADERS}>
    PRIVATE
        src/
)
