cmake_minimum_required(VERSION 3.0)
project(tools)

if (ON_TARGET)
    add_subdirectory(software_check)
    add_subdirectory(debug_apps)
    add_subdirectory(nvm_tools)
elseif()
    add_subdirectory(software_check)
endif()

include(GNUInstallDirs)
