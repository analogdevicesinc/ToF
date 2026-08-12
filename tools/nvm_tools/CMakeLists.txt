cmake_minimum_required(VERSION 3.0)
project(nvm_tools)

######################### Target based Options ################################
option(NXP "Set to ON when building on NXP" OFF)

# Add the device definition
if(NXP)
        add_definitions(-DNXP)
endif()


add_subdirectory(adsd3500_fw_update)
add_subdirectory(NVM_WRITE)

