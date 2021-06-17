cmake_minimum_required(VERSION 2.8)
project(examples)

if ( NOT NXP )
    add_subdirectory(aditof-demo)
    add_subdirectory(aditof-test)
endif()

add_subdirectory(first-frame)
add_subdirectory(data_collect)
if (WIN32)
    add_subdirectory(tof-viewer)
endif()

if (WITH_NETWORK)
    add_subdirectory(first-frame-network)
endif()

if (JETSON)
    add_subdirectory(imshow-jetson)
endif()

if (XAVIER)
    add_subdirectory(imshow-xavier)
endif()

include(GNUInstallDirs)

if ( NXP )
    install(PROGRAMS
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
        DESTINATION ${CMAKE_INSTALL_BINDIR} )

else()
    install(PROGRAMS
        $<TARGET_FILE:aditof-demo>
        $<TARGET_FILE:aditof-test>
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
	$<TARGET_FILE:ADIToFGUI>
        DESTINATION ${CMAKE_INSTALL_BINDIR} )
endif()
