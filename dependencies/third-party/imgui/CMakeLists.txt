cmake_minimum_required(VERSION 3.22)

if (NOT TARGET imgui)
	
    set(imguiextern_SOURCE_DIR imgui)
	set(gladextern_SOURCE_DIR glad)
    add_library(imgui STATIC
                ${imguiextern_SOURCE_DIR}/imgui.cpp
                ${imguiextern_SOURCE_DIR}/imgui_draw.cpp
                ${imguiextern_SOURCE_DIR}/imgui_widgets.cpp
				${imguiextern_SOURCE_DIR}/imgui_tables.cpp
                ${imguiextern_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
                ${imguiextern_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
				${gladextern_SOURCE_DIR}/src/gl.c
    )
	
	target_compile_definitions(imgui INTERFACE IMGUI_IMPL_OPENGL_LOADER_GLAD)

    target_include_directories(imgui PUBLIC
        ${imguiextern_SOURCE_DIR}
		${imguiextern_SOURCE_DIR}/examples/libs/glfw/include
        ${imguiextern_SOURCE_DIR}/backends
		${gladextern_SOURCE_DIR}/include
    )
	
else()
    message(STATUS "imgui is already a target. Skipping adding it twice")
endif()

add_library(imgui::imgui ALIAS imgui)
