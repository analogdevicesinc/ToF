file(GLOB SOURCES "*.c" "*.h")
file(GLOB HEADERS "../include/uvcgadget/*.h")

add_subdirectory(compat)

add_library(uvcgadget ${SOURCES})

set_target_properties(uvcgadget PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(uvcgadget
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/uvcgadget>
	$<INSTALL_INTERFACE:include>)

install(TARGETS uvcgadget
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include/uvcgadget)
