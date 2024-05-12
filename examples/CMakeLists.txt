cmake_minimum_required(VERSION 3.0)
project(examples)

add_subdirectory(first-frame)
add_subdirectory(data_collect)
add_subdirectory(test_frame_capture)
add_subdirectory(test_access_ini)
add_subdirectory(test_measure_fps)
add_subdirectory(test_getMetadata)
add_subdirectory(test_frameHandler)
add_subdirectory(test_saveCCBCFG)
add_subdirectory(test_frameAPIs)
add_subdirectory(test_get_camera_list)
add_subdirectory(test_initialize_load_config)
add_subdirectory(test_temp_compensation1)
add_subdirectory(test_temp_compensation2)
add_subdirectory(test_misc_APIs)
add_subdirectory(test_mipi_speed)
add_subdirectory(test_generic_template)
add_subdirectory(test_sensor_enum)
add_subdirectory(test_depth_sensor)
add_subdirectory(test_depth_sensor_control)

if (NOT NXP)
    add_subdirectory(tof-viewer)
endif()

include(GNUInstallDirs)

if (NXP)
    install(PROGRAMS
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
        $<TARGET_FILE:test_frame_capture>
	$<TARGET_FILE:test_access_ini>
	$<TARGET_FILE:test_measure_fps>
	$<TARGET_FILE:test_getMetadata>
	$<TARGET_FILE:test_frameHandler>
	$<TARGET_FILE:test_saveCCBCFG>
	$<TARGET_FILE:test_frameAPIs>
	$<TARGET_FILE:test_get_camera_list>
	$<TARGET_FILE:test_initialize_load_config>
	$<TARGET_FILE:test_temp_compensation1>
	$<TARGET_FILE:test_temp_compensation2>
	$<TARGET_FILE:test_misc_APIs>
	$<TARGET_FILE:test_mipi_speed>
	$<TARGET_FILE:test_generic_template>
	$<TARGET_FILE:test_sensor_enum>
	$<TARGET_FILE:test_depth_sensor>
	$<TARGET_FILE:test_depth_sensor_control>

        DESTINATION ${CMAKE_INSTALL_BINDIR} )
elseif(WIN32)
    install(PROGRAMS
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
	$<TARGET_FILE:test_frame_capture>
	$<TARGET_FILE:test_access_ini>
	$<TARGET_FILE:test_measure_fps>
	$<TARGET_FILE:test_getMetadata>
	$<TARGET_FILE:test_frameHandler>
	$<TARGET_FILE:test_saveCCBCFG>
	$<TARGET_FILE:test_frameAPIs>
	$<TARGET_FILE:test_get_camera_list>
	$<TARGET_FILE:test_initialize_load_config>
	$<TARGET_FILE:test_temp_compensation1>
	$<TARGET_FILE:test_temp_compensation2>
	$<TARGET_FILE:test_misc_APIs>
	$<TARGET_FILE:test_mipi_speed>
	$<TARGET_FILE:test_generic_template>
	$<TARGET_FILE:test_sensor_enum>
	$<TARGET_FILE:test_depth_sensor>
	$<TARGET_FILE:test_depth_sensor_control>
        $<TARGET_FILE:ADIToFGUI>
        DESTINATION ${CMAKE_INSTALL_BINDIR} )
endif()
