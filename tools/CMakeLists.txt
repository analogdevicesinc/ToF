cmake_minimum_required(VERSION 3.0)
project(tools)

if (NXP OR NVIDIA)
    add_subdirectory(nvm_tools)
endif()

add_subdirectory(softwarecheck)

include(GNUInstallDirs)
