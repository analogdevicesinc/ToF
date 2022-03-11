file(GLOB SOURCES "*.c" "*.h")
file(GLOB HEADERS "../include/uvcgadget/*.h")

add_library(uvcgadget ${SOURCES})

set_target_properties(uvcgadget PROPERTIES PUBLIC_HEADER "${HEADERS}")

target_include_directories(uvcgadget
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/apps/uvc-app/include>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/apps/uvc-app/include/uvcgadget>)
