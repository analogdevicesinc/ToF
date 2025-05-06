cmake_minimum_required(VERSION 3.22)

if (NOT TARGET imgui)
	
    set(imguiextern_SOURCE_DIR imgui)
	set(gladextern_SOURCE_DIR glad)
    set(imguitoggleextern_SOURCE_DIR imgui_toggle)
    set(implotextern_SOURCE_DIR implot)
    add_library(imgui STATIC
                ${imguiextern_SOURCE_DIR}/imgui.cpp
                ${imguiextern_SOURCE_DIR}/imgui_draw.cpp
                ${imguiextern_SOURCE_DIR}/imgui_widgets.cpp
				${imguiextern_SOURCE_DIR}/imgui_tables.cpp
                ${imguiextern_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
                ${imguiextern_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
                ${imguitoggleextern_SOURCE_DIR}/imgui_toggle.cpp
				${imguitoggleextern_SOURCE_DIR}/imgui_toggle_palette.cpp
				${imguitoggleextern_SOURCE_DIR}/imgui_toggle_presets.cpp
				${imguitoggleextern_SOURCE_DIR}/imgui_toggle_renderer.cpp
                ${implotextern_SOURCE_DIR}/implot.cpp
                ${implotextern_SOURCE_DIR}/implot_items.cpp
				${gladextern_SOURCE_DIR}/src/gl.c
    )
	
	target_compile_definitions(imgui INTERFACE IMGUI_IMPL_OPENGL_LOADER_GLAD)

    target_include_directories(imgui PUBLIC
        ${imguiextern_SOURCE_DIR}
		${imguiextern_SOURCE_DIR}/examples/libs/glfw/include
        ${imguiextern_SOURCE_DIR}/backends
		${imguitoggleextern_SOURCE_DIR}
        {${implotextern_SOURCE_DIR}
		${gladextern_SOURCE_DIR}/include
    )
	
else()
    message(STATUS "imgui is already a target. Skipping adding it twice")
endif()

add_library(imgui::imgui ALIAS imgui)
