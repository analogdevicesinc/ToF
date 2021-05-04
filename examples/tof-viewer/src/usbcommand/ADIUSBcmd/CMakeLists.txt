
# add_library(ADIUSBcmd usbcommand.c)
add_library(ADIUSBcmd
    STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/usbcommand.c"
	)
	
file(GLOB_RECURSE CFILES "${CMAKE_SOURCE_DIR}/*.c")
SET_SOURCE_FILES_PROPERTIES(${CFILES} PROPERTIES LANGUAGE CXX )

target_include_directories(ADIUSBcmd PUBLIC 
                            ${CMAKE_CURRENT_SOURCE_DIR}
							${PROJECT_SOURCE_DIR}/src
							${PROJECT_SOURCE_DIR}/external/libusb/src/libusb
							)
