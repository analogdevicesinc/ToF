cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

# Install the helper header for opencv in include/aditof
include(GNUInstallDirs)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/aditof_opencv.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aditof
)

if(OPENCV_FOUND OR
   OPENCV_CORE_FOUND) # we check for this because the OpenCVConfig.cmake(v2.4.9.1) used on dragonboards fails to set OPENCV_FOUND

    if(OpenCV_VERSION VERSION_LESS "3.0.0")
        message(STATUS "USING OPENCV Version 2")
        add_definitions(-DOPENCV2)
    else()
        message(STATUS "USING OPENCV Version 3")
    endif()
endif()

add_subdirectory(imshow)
