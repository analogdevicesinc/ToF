cmake_minimum_required(VERSION 3.0)
project(tools)

if (NXP OR NVIDIA OR RPI)
    add_subdirectory(nvm_tools)
endif()

add_subdirectory(software_check)

include(GNUInstallDirs)
