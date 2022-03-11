file(GLOB SOURCES "*.c" "*.h")
file(GLOB HEADERS "../include/uvcgadget/*.h")

add_subdirectory(compat)

add_library(uvcgadget STATIC ${SOURCES})

set_target_properties(uvcgadget PROPERTIES PUBLIC_HEADER "${HEADERS}")
message(STATUS "foo include dir: ${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(uvcgadget
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}../include>
		${CMAKE_CURRENT_SOURCE_DIR}../include/uvcgadget>)
