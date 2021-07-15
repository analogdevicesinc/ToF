cmake_minimum_required(VERSION 2.8)
project(apps)

if (NXP)
        if (WITH_NETWORK)
                add_subdirectory(server)
		add_subdirectory(uvc-app)
	endif()
endif()
