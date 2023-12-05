cmake_minimum_required(VERSION 3.0)
project(apps)

if (ON_TARGET AND WITH_PROTOBUF_DEPENDENCY)
        add_subdirectory(uvc-app)
        if (WITH_NETWORK)
                add_subdirectory(server)
	endif()
endif()
