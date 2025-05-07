#include <glad/gl.h>
#include <cmath>
#include <numeric>
#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"
#include "implot.h"
#include "imoguizmo.hpp"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace adiMainWindow;

static std::vector<float> depth_line_values;

//*******************************************
//* Section: Generic
//*******************************************

void ADIMainWindow::RenderFrameHoverInfo(ImVec2 hoveredImagePixel,
    uint16_t* currentImage,
    int imageWidth,
    bool isHovered,
    ADI_Image_Format_t format,
    std::string units) {

    if (static_cast<int>(hoveredImagePixel.x) ==
        static_cast<int>(m_invalid_hovered_pixel.x) &&
        static_cast<int>(hoveredImagePixel.y) ==
        static_cast<int>(m_invalid_hovered_pixel.y)) {
        return;
    }

    if (hoveredImagePixel.x < 0 || hoveredImagePixel.y < 0)
        return;

    uint16_t pixelValue = currentImage[((int)hoveredImagePixel.y * (imageWidth)) + int(hoveredImagePixel.x)]; //153280 is pixel value linear
    std::string imageType = "Unknown";
    ImGuiWindowFlags overlayFlags2 =
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar;

    ImGui::SetNextWindowPos(ImGui::GetCursorScreenPos(), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.3f); // Transparent background
    ImGui::Begin("hover", nullptr, overlayFlags2);

    if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16) {
        imageType = "Depth";
    } else if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_AB16) {
        imageType = "AB";
    }

    if (isHovered || ImGui::IsWindowHovered()) {
        ImGui::Text("%s: %d, %d, %d %s", imageType.c_str(), int(hoveredImagePixel.x), int(hoveredImagePixel.y), pixelValue, units.c_str());
    }
    else {
        ImGui::Text("Hover over the image to get pixel value");
    }
    ImGui::End();
}

void ADIMainWindow::GetHoveredImagePix(ImVec2& hoveredImagePixel,
    ImVec2 imageStartPos, ImVec2 mousePos,
    ImVec2 display_depth_dimensions,
    ImVec2 source_depth_image_dimensions) {

    ImVec2 hoveredUIPixel;
    hoveredUIPixel.x = mousePos.x - imageStartPos.x;
    hoveredUIPixel.y = mousePos.y - imageStartPos.y;    

    ImVec2 _displayDepthDimensions = display_depth_dimensions;
    ImVec2 _sourceDepthImageDimensions = source_depth_image_dimensions;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(_sourceDepthImageDimensions.x, _sourceDepthImageDimensions.y);
    }

    // Do not show out values when cursor is not over image - ie out of bounds.
    if (hoveredUIPixel.x > _displayDepthDimensions.x ||
        hoveredUIPixel.y > _displayDepthDimensions.y || hoveredUIPixel.x < 0 ||
        hoveredUIPixel.y < 0) {
        hoveredImagePixel.x = -1;
        hoveredImagePixel.y = -1;
        return;
    }

    hoveredUIPixel.x = (std::max)((std::min)(hoveredUIPixel.x, _displayDepthDimensions.x), 0.0f);
    hoveredUIPixel.y = (std::max)((std::min)(hoveredUIPixel.y, _displayDepthDimensions.y), 0.0f);

	// Scale the hovered pixel to the image pixel co-ordinate system
    const float uiCoordinateToImageCoordinateRatio = _sourceDepthImageDimensions.x / _displayDepthDimensions.x;
    hoveredImagePixel.x = std::round(hoveredUIPixel.x * uiCoordinateToImageCoordinateRatio);
    hoveredImagePixel.y = std::round(hoveredUIPixel.y * uiCoordinateToImageCoordinateRatio);

    if (rotationangledegrees == 90) {

        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.y =
            m_source_depth_image_dimensions.y - hoveredImagePixel.y;
    }
    else if (rotationangledegrees == 270) {

        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.x =
            m_source_depth_image_dimensions.x - hoveredImagePixel.x;
    }
    else if (rotationangledegrees == 180) {

        hoveredImagePixel.x =
            _sourceDepthImageDimensions.x - hoveredImagePixel.x;
        hoveredImagePixel.y =
            _sourceDepthImageDimensions.y - hoveredImagePixel.y;
    }
}

float ADIMainWindow::DisplayFrameWindow(ImVec2 windowSize, ImVec2 &displayUpdate,
                                       ImVec2 &size) {

    float autoscale = std::fmin((windowSize.x / m_view_instance->frameWidth),
                                (windowSize.y / m_view_instance->frameHeight));

    size.x = m_view_instance->frameWidth * autoscale;
    size.y = m_view_instance->frameHeight * autoscale;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(size.x, size.y);
    }

    displayUpdate = {static_cast<float>(size.x), static_cast<float>(size.y)};

    size.x /= m_dpi_scale_factor;
    size.y /= m_dpi_scale_factor;

    return autoscale;
}

void ADIMainWindow::synchronizeVideo() {
    m_view_instance->m_capturedFrame = m_view_instance->m_ctrl->getFrame();

    aditof::FrameDetails frameDetails;
    m_view_instance->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(m_view_instance->m_frameCapturedMutex);

    m_view_instance->m_depthFrameAvailable = true;
    m_view_instance->m_abFrameAvailable = true;
    m_view_instance->m_pointCloudFrameAvailable = true;
    m_view_instance->numOfThreads = 3;

    m_view_instance->frameHeight = frameDetails.height;
    m_view_instance->frameWidth = frameDetails.width;

    lock.unlock();

    m_view_instance->m_frameCapturedCv.notify_all();

    if (!m_off_line) {
        m_view_instance->m_ctrl->requestFrame();
    } else {
        m_view_instance->m_ctrl->requestFrame();
        m_view_instance->m_ctrl->requestFrame(m_off_line_frame_index);
    }

    /*********************************/
    std::unique_lock<std::mutex> imshow_lock(m_view_instance->m_imshowMutex);
    m_view_instance->m_barrierCv.wait(imshow_lock, [&]() {
        return m_view_instance->m_waitKeyBarrier == m_view_instance->numOfThreads;
    });
    m_view_instance->m_waitKeyBarrier = 0;
    /*********************************/
}

void ADIMainWindow::DepthLinePlot(ImGuiWindowFlags overlayFlags) {

	if (depth_line_values.size() == 0) {
		return;
	}

    SetWindowPosition(m_dict_win_position["plotA"].x, m_dict_win_position["plotA"].y);
    SetWindowSize(m_dict_win_position["plotA"].width, m_dict_win_position["plotA"].height);

    if (ImGui::Begin("Depth Pixel Plot", nullptr, overlayFlags | ImGuiWindowFlags_NoTitleBar)) {
        ImPlot::SetNextAxesToFit();
        if (ImPlot::BeginPlot("Line Depth Values")) {

            std::vector<float> x(depth_line_values.size());

            std::iota(x.begin(), x.end(), 0);
            ImPlot::PlotLine("Depth over Line", x.data(), depth_line_values.data(), depth_line_values.size());
            ImPlot::EndPlot();
        }
        ImGui::End();
    }
}

//*******************************************
//* Section: Handling of AB Window 
//*******************************************

void ADIMainWindow::InitOpenGLABTexture() {
    /********************************************/
    //ab Texture
    GLuint ab_texture;
    glGenTextures(1, &ab_texture);
    glBindTexture(GL_TEXTURE_2D, ab_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    m_gl_ab_video_texture = ab_texture;
    /*************************************************/
}

void ADIMainWindow::DisplayActiveBrightnessWindow(
    ImGuiWindowFlags overlayFlags) {

    ImVec2 size;

    auto imageScale =
        DisplayFrameWindow(ImVec2(m_ab_position->width, m_ab_position->height),
                           m_display_ab_dimensions, size);

    SetWindowPosition(m_ab_position->x, m_ab_position->y);
    SetWindowSize(m_ab_position->width, m_ab_position->height);

    if (ImGui::Begin("Active Brightness Window", nullptr, overlayFlags | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::SetCursorPos(ImVec2(0, 0));

        if (m_view_instance->ab_video_data_8bit != nullptr) {
            glBindTexture(GL_TEXTURE_2D, m_gl_ab_video_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_view_instance->frameWidth,
                         m_view_instance->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                         m_view_instance->ab_video_data_8bit);
            glad_glGenerateMipmap(GL_TEXTURE_2D);

            ImVec2 _displayABDimensions = m_display_ab_dimensions;

            if (rotationangledegrees == 90 || rotationangledegrees == 270) {
                std::swap(_displayABDimensions.x, _displayABDimensions.y);
            }

            ImageRotated((ImTextureID)m_gl_ab_video_texture,
                         ImVec2(m_ab_position->width, m_ab_position->height),
                         ImVec2(_displayABDimensions.x, _displayABDimensions.y),
                         rotationangleradians);
        }

        ImVec2 hoveredImagePixel = m_invalid_hovered_pixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
            ImGui::GetIO().MousePos, m_display_ab_dimensions, m_source_depth_image_dimensions);
		RenderFrameHoverInfo(hoveredImagePixel, 
            m_view_instance->ab_video_data,
            m_view_instance->frameWidth,
            ImGui::IsWindowHovered(),
            ADI_Image_Format_t::ADI_IMAGE_FORMAT_AB16, "Intensity");
    }

    ImGui::End();
}

//*******************************************
//* Section: Handling of Depth Widow
//*******************************************

void ADIMainWindow::InitOpenGLDepthTexture() {
    depth_line_values.clear();
    /********************************************/
    //Depth Texture
    GLuint depth_texture;
    glGenTextures(1, &depth_texture);
    glBindTexture(GL_TEXTURE_2D, depth_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    m_gl_depth_video_texture = depth_texture;
    /*************************************************/
}

static std::vector<ImVec2> GetLinePixels(int x0, int y0, int x1, int y1) {
    std::vector<ImVec2> points;

    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        points.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }

    return points;
}

void ADIMainWindow::DisplayDepthWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    auto imageScale = DisplayFrameWindow(
        ImVec2(m_depth_position->width, m_depth_position->height),
        m_display_depth_dimensions, size);

    m_source_depth_image_dimensions = { (float)(m_view_instance->frameWidth),
                                  (float)(m_view_instance->frameHeight) };


    SetWindowPosition(m_depth_position->x, m_depth_position->y);
    SetWindowSize(m_depth_position->width, m_depth_position->height);

    std::string title = "Depth Window";
    if (ImGui::Begin(title.c_str(), nullptr, overlayFlags | ImGuiWindowFlags_NoTitleBar)) {

        ImGui::SetCursorPos(ImVec2(0, 0));

        if (m_view_instance->depth_video_data_8bit != nullptr) {
            glBindTexture(GL_TEXTURE_2D, m_gl_depth_video_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_view_instance->frameWidth,
                         m_view_instance->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                         m_view_instance->depth_video_data_8bit);
            glad_glGenerateMipmap(GL_TEXTURE_2D);
            //delete m_view_instance->depth_video_data_8bit;

            ImVec2 _displayDepthDimensions = m_display_depth_dimensions;

            if (rotationangledegrees == 90 || rotationangledegrees == 270) {
                std::swap(_displayDepthDimensions.x, _displayDepthDimensions.y);
            }

            ImageRotated(
                (ImTextureID)m_gl_depth_video_texture,
                ImVec2(m_depth_position->width, m_depth_position->height),
                ImVec2(_displayDepthDimensions.x, _displayDepthDimensions.y),
                rotationangleradians);
        }

		std::vector<ImVec2> depth_line_points;

        static std::vector<std::pair<float, float>> depthLine;
        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            ImVec2 mousePos = ImGui::GetMousePos();
            if (depthLine.size() >= 2) {
                depthLine.clear();
                depth_line_points.clear();
            } else {
                depthLine.push_back({ mousePos.x, mousePos.y });
            }
        }

		if (depthLine.size() == 1) {
            ImVec2 mousePos = ImGui::GetMousePos();
			ImVec2 start = ImVec2(depthLine[0].first, depthLine[0].second);
			ImVec2 end = ImVec2(mousePos.x, mousePos.y);
			ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 2.0f);
		} else if (depthLine.size() >= 2) {
            ImVec2 start;
            ImVec2 end;

            if (depthLine[0].first < depthLine[1].first) {
                start = ImVec2(depthLine[0].first, depthLine[0].second);
                end = ImVec2(depthLine[1].first, depthLine[1].second);
            }
            else {
                end = ImVec2(depthLine[0].first, depthLine[0].second);
                start = ImVec2(depthLine[1].first, depthLine[1].second);
            }

            ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 2.0f);

            ImVec2 _start = m_invalid_hovered_pixel;
			ImVec2 _end = m_invalid_hovered_pixel; 

            GetHoveredImagePix(_start, ImGui::GetCursorScreenPos(),
                start, m_display_depth_dimensions, m_source_depth_image_dimensions);

            GetHoveredImagePix(_end, ImGui::GetCursorScreenPos(),
                end, m_display_depth_dimensions, m_source_depth_image_dimensions);

            depth_line_points = GetLinePixels(_start.x, _start.y, _end.x, _end.y);

            depth_line_values.clear();
			for (const auto& point : depth_line_points) {
                uint32_t offset = m_view_instance->frameWidth * point.y + point.x;
                float depth = m_view_instance->depth_video_data[offset];

                if (depth == 0 && offset > 0)
                    depth = m_view_instance->depth_video_data[offset - 1];

				depth_line_values.emplace_back(depth);
			}
        }

        ImVec2 hoveredImagePixel = m_invalid_hovered_pixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
                           ImGui::GetIO().MousePos, m_display_depth_dimensions, m_source_depth_image_dimensions);
        RenderFrameHoverInfo(
            hoveredImagePixel, 
            m_view_instance->depth_video_data,
            m_view_instance->frameWidth, 
            ImGui::IsWindowHovered(),
            ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16, "mm");
    }

    ImGui::End();
}

//*******************************************
//* Section: Handling of Point Cloud Window
//*******************************************

void ADIMainWindow::DisplayPointCloudWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    auto imageScale =
        DisplayFrameWindow(ImVec2(m_pc_position->width, m_pc_position->height),
                           m_display_point_cloud_dimensions, size);

    SetWindowPosition(m_pc_position->x, m_pc_position->y);
    SetWindowSize(m_pc_position->width, m_pc_position->height);

    if (ImGui::Begin("Point Cloud Window", nullptr, overlayFlags)) {

        ImGui::SetCursorPos(ImVec2(0, 0));

        ProcessInputs(window);

        PreparePointCloudVertices(m_view_instance->vertexBufferObject,
                                  m_view_instance->vertexArrayObject);

        glad_glBindFramebuffer(GL_FRAMEBUFFER, m_gl_framebuffer);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPointSize(m_point_size);

        // draw our Image
        glUseProgram(m_view_instance->pcShader.Id());
        mat4x4_perspective(m_projection_mat, Radians(m_field_of_view),
                           (float)m_view_instance->frameWidth /
                               (float)m_view_instance->frameHeight,
                            0.1f, 100.0f);
        glUniformMatrix4fv(m_view_instance->projectionIndex, 1, GL_FALSE,
                            &m_projection_mat[0][0]);

        //Look-At function[ x, y, z] = (m_camera_position_vec, m_camera_position_vec + m_camera_front_vec, m_camera_up_vec);
        vec3_add(m_camera_position_front_vec, m_camera_position_vec, m_camera_front_vec);
        mat4x4_look_at(m_view_mat, m_camera_position_vec, m_camera_position_front_vec, m_camera_up_vec);
        glUniformMatrix4fv(m_view_instance->viewIndex, 1, GL_FALSE, &m_view_mat[0][0]);
        glUniformMatrix4fv(m_view_instance->modelIndex, 1, GL_FALSE, &m_model_mat[0][0]);

        glBindVertexArray(
            m_view_instance->vertexArrayObject); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        glDrawArrays(GL_POINTS, 0,
                     static_cast<GLsizei>(m_view_instance->vertexArraySize));
        glBindVertexArray(0);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDisable(GL_DEPTH_TEST);

        ImVec2 _displayPointCloudDimensions = m_display_point_cloud_dimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayPointCloudDimensions.x,
                        _displayPointCloudDimensions.y);
        }

        ImageRotated((ImTextureID)m_gl_pointcloud_video_texture,
                        ImVec2(m_pc_position->width, m_pc_position->height),
                        ImVec2(_displayPointCloudDimensions.x,
                            _displayPointCloudDimensions.y),
                        rotationangleradians);
        glDeleteVertexArrays(1, &m_view_instance->vertexArrayObject);
        glDeleteBuffers(1, &m_view_instance->vertexBufferObject);


        const char *col1Text = "Camera Pos";
        const float padding = 20.0f; // Optional extra space
        float col1Width = ImGui::CalcTextSize(col1Text).x + padding;
#if 0
        if (ImGui::BeginTable("Information Table", 2)) {
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed,
                                    col1Width);
            ImGui::TableSetupColumn("Value",
                                    ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("FoV");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%0.2f", m_field_of_view);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Camera Pos");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("(%0.2f, %0.2f, %0.2f)", m_camera_position_vec[0], m_camera_position_vec[1],
                        m_camera_position_vec[2]);

            ImGui::EndTable();
        }
#endif
        float viewMatrix[16];
        float projMatrix[16];

        glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 1000.0f);
        memcpy(projMatrix, glm::value_ptr(proj), sizeof(float) * 16);

        memcpy(viewMatrix, m_model_mat, sizeof(float) * 16);
        ImOGuizmo::SetRect(m_pc_position->x + 5.0f, m_pc_position->y + 15.0f, 50.0f);
        ImOGuizmo::DrawGizmo(viewMatrix, projMatrix, 10.0f);
    }
    ImGui::End();
}

void ADIMainWindow::PreparePointCloudVertices(unsigned int &vbo,
                                              unsigned int &vao) {

    glGenVertexArrays(1, &vao);
    //Initialize Point Cloud Image
    glGenBuffers(1, &vbo);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(vao);
    //Bind Point Cloud Image
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //Pass on Image buffer here:
    glBufferData(GL_ARRAY_BUFFER, m_view_instance->vertexArraySize,
                 m_view_instance->normalized_vertices,
                 GL_DYNAMIC_DRAW);

    //Image
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
        (void *)0); //Our image buffer got bigger: [X, Y, Z, R, G, B]
    glEnableVertexAttribArray(0); //Enable [X, Y, Z]

    //Point Cloud Color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1); //Enable [R, G, B]

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}

void ADIMainWindow::InitOpenGLPointCloudTexture() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE); //Enable point size feature

    constexpr char const pointCloudVertexShader[] =
        R"(
				#version 330 core
				layout (location = 0) in vec3 aPos;
				layout (location = 1) in vec3 hsvColor;//Contains R, G, B, values based on HSV standard

				uniform mat4 model;
				uniform mat4 view;
				uniform mat4 projection;

				out vec4 color_based_on_position;

				void main()
				{
					gl_Position = projection * view * model * vec4(aPos, 1.0);
					color_based_on_position = vec4(hsvColor, 1.0);
				}
				)";

    constexpr char const pointCloudFragmentShader[] =
        R"(
				#version 330 core
				out vec4 FragColor;
				in vec4 color_based_on_position;
				void main()
				{
					FragColor = color_based_on_position;
				}
				)";

    //Build and compile our shaders
    adiviewer::ADIShader vertexShader(
        GL_VERTEX_SHADER,
        pointCloudVertexShader); //Our vertices (whole image)
    adiviewer::ADIShader fragmentShader(GL_FRAGMENT_SHADER,
                                        pointCloudFragmentShader); //Color map
    m_view_instance->pcShader.CreateProgram();
    m_view_instance->pcShader.AttachShader(std::move(vertexShader));
    m_view_instance->pcShader.AttachShader(std::move(fragmentShader));
    m_view_instance->pcShader.Link();

    //Get the model, view, and projection index
    m_view_instance->modelIndex = glGetUniformLocation(m_view_instance->pcShader.Id(), "model");
    m_view_instance->viewIndex = glGetUniformLocation(m_view_instance->pcShader.Id(), "view");
    m_view_instance->projectionIndex =
        glGetUniformLocation(m_view_instance->pcShader.Id(), "projection");

    //Initialize Model, View, and Projection Matrices to Identity
    mat4x4_identity(m_view_mat);
    mat4x4_identity(m_projection_mat);
    mat4x4_identity(m_model_mat);

    //Create Frame Buffers to be able to display on the Point Cloud Window.
    glad_glGenFramebuffers(1, &m_gl_framebuffer);
    glad_glBindFramebuffer(GL_FRAMEBUFFER, m_gl_framebuffer);
    // create a color attachment texture
    glGenTextures(1, &m_gl_pointcloud_video_texture);
    glBindTexture(GL_TEXTURE_2D, m_gl_pointcloud_video_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_main_window_width, m_main_window_height, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glad_glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                GL_TEXTURE_2D,
                           m_gl_pointcloud_video_texture, 0);
    glad_glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

static inline ImVec2 operator+(const ImVec2 &lhs, const ImVec2 &rhs) {
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}

static inline ImVec2 operator/(const ImVec2 &lhs, const float rhs) {
    return ImVec2(lhs.x / rhs, lhs.y / rhs);
}

static inline ImVec2 operator*(const ImVec2 &lhs, const float rhs) {
    return ImVec2(lhs.x * rhs, lhs.y * rhs);
}

ImVec2 ADIMainWindow::ImRotate(const ImVec2 &v, float cos_a, float sin_a) {
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}

void ADIMainWindow::ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size,
                                 float angle) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    ImVec2 _center =
        ((center * m_dpi_scale_factor) / 2.0f) + ImGui::GetCursorScreenPos();
    ImVec2 _size = size;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    ImVec2 pos[4] = {
        _center +
            ImRotate(ImVec2(-_size.x * 0.5f, -_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(_size.x * 0.5f, -_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(_size.x * 0.5f, +_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(-_size.x * 0.5f, +_size.y * 0.5f), cos_a, sin_a)};
    ImVec2 uvs[4] = {ImVec2(0.0f, 0.0f), ImVec2(1.0f, 0.0f), ImVec2(1.0f, 1.0f),
                     ImVec2(0.0f, 1.0f)};

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0],
                            uvs[1], uvs[2], uvs[3], IM_COL32_WHITE);
}

void ADIMainWindow::PointCloudReset() {

    const mat4x4 m_view_default =
    { {1.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f},
      {-0.0213157870, -0.00631578919, -3.0f, 1.0f} };

    const mat4x4 m_projection_default = 
    { {14.3006659f, 0.0f, 0.0f, 0.0f},
      {0.0f, 14.3006659f, 0.0f, 0.0f},
      {0.0f, 0.0f, -1.00200200f, -1.0f},
      {0.0f, 0.0f, -0.200200200f, 0.0f} };

    memcpy(m_view_mat, m_view_default, sizeof(m_view_mat));
    memcpy(m_projection_mat, m_projection_default, sizeof(m_projection_mat));
    mat4x4_identity(m_model_mat);
    m_delta_time = 0.1;
    m_field_of_view = 8.0f;
    m_view_instance->Max_X = 6000.0;
    m_view_instance->Max_Y = 6000.0;
    m_view_instance->Max_Z = 6000.0;

    m_camera_position_vec[0] = 0.0213157870f;
    m_camera_position_vec[1] = 0.00631578919f;
    m_camera_position_vec[2] = 3.0f;
    m_camera_front_vec[0] = 0.0;
    m_camera_front_vec[1] = 0.0;
    m_camera_front_vec[2] = -1.0;
    m_camera_up_vec[0] = 0.0;
    m_camera_up_vec[1] = 1.0;
    m_camera_up_vec[2] = 0.0;
    m_point_size = 1;
}

float ADIMainWindow::Radians(float degrees) {
    float radians = (IM_PI / 180) * degrees;
    return radians;
}

void ADIMainWindow::ProcessInputs(GLFWwindow *window) {
    //Sensitivity
    const float maxFov = 45.0f;
    float cameraSpeed = 2.5f * m_delta_time;

    ImGuiIO &io = ImGui::GetIO(); //Get mouse events

    if (ImGui::IsWindowHovered()) {
        if (io.MouseWheel) //Use mouse wheel to scroll up/down for zoom in/zoom out
        {
            m_field_of_view -= (float)io.MouseWheel;
            if (m_field_of_view < 1.0f) {
                m_field_of_view = 1.0f;
            }
            if (m_field_of_view > maxFov) {
                m_field_of_view = maxFov;
            }
        }
    }

    MouseMovementType movementType = MouseMovementType::None;

    ImVec2 mouseDownPos(-1.f, -1.f);
    //Press the left button for image rotation
    if (io.MouseDown[GLFW_MOUSE_BUTTON_1]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_1];
        movementType = MouseMovementType::Rotation;
        m_mouse_down = true;
    }
    //Press the right button for image translation
    else if (io.MouseDown[GLFW_MOUSE_BUTTON_2]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_2];
        movementType = MouseMovementType::Translation;
        m_mouse_down = true;
    } else {
        m_mouse_down = false;
    }

    if (m_mouse_down) {
        const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
        // Normalize to the image start coordinates
        //
        mouseDownPos.x -= imageStartPos.x;
        mouseDownPos.y -= imageStartPos.y;

        const vec2 mouseDelta{io.MouseDelta.x, io.MouseDelta.y};
        const vec2 displayDimensions{static_cast<float>(m_main_window_width),
                                     static_cast<float>(m_main_window_height)};
        // Only count drags if they originated on the image
        //
        if (mouseDownPos.x >= 0.f &&
            mouseDownPos.x <= m_display_point_cloud_dimensions.x &&
            mouseDownPos.y >= 0.f &&
            mouseDownPos.y <= m_display_point_cloud_dimensions.y) {
            if (movementType == MouseMovementType::Rotation) {
                const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
                const vec2 mousePos{io.MousePos.x + imageStartPos.x,
                                    io.MousePos.y + imageStartPos.y};
                vec2 lastMousePos;

                vec2_copy(lastMousePos, mousePos);
                vec2_sub(lastMousePos, mousePos, mouseDelta);

                quat newRotationQuat;
                GetArcballRotation(newRotationQuat, displayDimensions,
                                   lastMousePos, mousePos);

                mat4x4 newRotationMtx;
                mat4x4_from_quat(newRotationMtx, newRotationQuat);

                MatrixMultiply(m_model_mat, newRotationMtx, m_model_mat);
            }
            if (movementType == MouseMovementType::Translation) {
                m_camera_position_vec[0] -=
                    mouseDelta[0] * m_translation_sensitivity / (maxFov - m_field_of_view + 1);
                m_camera_position_vec[1] -=
                    mouseDelta[1] * m_translation_sensitivity / (maxFov - m_field_of_view + 1);
            }
        }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_W)) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, m_camera_front_vec, cameraSpeed);
        vec3_add(m_camera_position_vec, m_camera_position_vec, aux);
    }

    if (ImGui::IsKeyPressed(ImGuiKey_S)) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, m_camera_front_vec, cameraSpeed);
        vec3_sub(m_camera_position_vec, m_camera_position_vec, aux);
    }

    if (ImGui::IsKeyPressed(ImGuiKey_A)) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, m_camera_front_vec, m_camera_up_vec);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_sub(m_camera_position_vec, m_camera_position_vec, aux);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_D)) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, m_camera_front_vec, m_camera_up_vec);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_add(m_camera_position_vec, m_camera_position_vec, aux);
    }
}

void ADIMainWindow::GetArcballRotation(quat rotation,
                                       const vec2 displayDimensions,
                                       const vec2 startPos, const vec2 endPos) {
    vec3 startVector;
    MapToArcball(startVector, displayDimensions, startPos);

    vec3 endVector;
    MapToArcball(endVector, displayDimensions, endPos);

    vec3 cross;
    vec3_mul_cross(cross, startVector, endVector);

    constexpr float epsilon = 0.001f;
    if (vec3_len(cross) < epsilon) {
        // Smooth out floating point error if the user didn't move the mouse
        // enough that it should register
        //
        quat_identity(rotation);
    } else {
        // The first 3 elements of the quaternion are the unit vector perpendicular
        // to the rotation (i.e. the cross product); the last element is the magnitude
        // of the rotation
        //
        vec3_copy(rotation, cross);
        vec3_norm(cross, cross);
        rotation[3] = vec3_mul_inner(startVector, endVector);
    }
}

void ADIMainWindow::MapToArcball(vec3 out, const vec2 displayDimensions,
                                 const vec2 mousePos) {
    // Scale coords to (-1, 1) to simplify some of the math
    //
    vec2 scaledMousePos;
    for (int i = 0; i < 2; ++i) {
        scaledMousePos[i] =
            mousePos[i] * (1.0f / ((displayDimensions[i] - 1.0f) * 0.5f)) -
            1.0f;
    }

    float lenSquared = scaledMousePos[0] * scaledMousePos[0] +
                       scaledMousePos[1] * scaledMousePos[1];

    // If the point is 'outside' our virtual sphere, we need to normalize to the sphere
    // This works because our sphere is of radius 1
    //
    if (lenSquared > 1.f) {
        const float normalizationFactor = 1.f / std::sqrt(lenSquared);

        // Return a point on the edge of the sphere
        //
        out[0] = scaledMousePos[0] * normalizationFactor;
        out[1] = scaledMousePos[1] * normalizationFactor;
        out[2] = 0.f;
    } else {
        // Return a point inside the sphere
        //
        out[0] = scaledMousePos[0];
        out[1] = scaledMousePos[1];
        out[2] = std::sqrt(1.f - lenSquared);
    }
}

void ADIMainWindow::MatrixMultiply(mat4x4 out, mat4x4 a, mat4x4 b) {
    mat4x4 atmp;
    mat4x4 btmp;
    mat4x4_dup(atmp, a);
    mat4x4_dup(btmp, b);
    mat4x4_mul(out, a, b);
}