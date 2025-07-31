cmake_minimum_required(VERSION 3.0)
project(nvm_tools)

######################### Target based Options ################################
option(NXP "Set to ON when building on NXP" OFF)
option(NVIDIA "Set to ON when building on NVIDIA" OFF)

# Add the device definition
if(NXP)
	add_definitions(-DNXP)
elseif(NVIDIA)
	add_definitions(-DNVIDIA)
endif()

add_subdirectory(NVM_WRITE)
add_subdirectory(NVM_READ)
add_subdirectory(CCB_WRITE)
add_subdirectory(CCB_READ)
add_subdirectory(adsd3500_fw_update)

