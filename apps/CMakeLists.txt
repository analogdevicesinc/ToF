cmake_minimum_required(VERSION 3.0)
project(apps)

if (NXP)
        add_subdirectory(uvc-app)
        if (WITH_NETWORK)
                add_subdirectory(server)
	endif()
endif()
