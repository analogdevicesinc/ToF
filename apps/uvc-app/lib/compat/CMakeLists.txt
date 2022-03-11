include(CheckFunctionExists)
include(CheckIncludeFile)

check_include_file(dirent.h HAVE_DIRENT_H)

check_function_exists(glob HAVE_GLOB)
if(NOT HAVE_GLOB)
	set(SOURCES "${SOURCES}"
	    "${CMAKE_CURRENT_SOURCE_DIR}/glob.c"
	    "${CMAKE_SOURCE_DIR}/include/compat/glob.h"
	    PARENT_SCOPE)
endif()
