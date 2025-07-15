#include <glad/gl.h>
#include <cmath>
#include <numeric>
#include <fstream>
#include <string>
#include <stdexcept>
#include <aditof/frame_handler.h>
#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"
#include "implot.h"
#include "imoguizmo.hpp"
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#undef NDEBUG
#include <cassert>


using namespace adiMainWindow;

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

    size.x = m_view_instance->frameWidth * autoscale * m_dpi_scale_factor;
    size.y = m_view_instance->frameHeight * autoscale * m_dpi_scale_factor;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(size.x, size.y);
    }

    displayUpdate = {static_cast<float>(size.x), static_cast<float>(size.y)};

    size.x /= m_dpi_scale_factor;
    size.y /= m_dpi_scale_factor;

    return autoscale;
}

int32_t ADIMainWindow::synchronizeVideo(std::shared_ptr<aditof::Frame>& frame) {
    auto tmpFrame = m_view_instance->m_ctrl->getFrame();
	if (tmpFrame != nullptr) {
        m_view_instance->m_capturedFrame = tmpFrame;
	}

    if (m_view_instance->m_capturedFrame == nullptr) {
        return -1;
    }

    frame = m_view_instance->m_capturedFrame;

    aditof::FrameDetails frameDetails;
    m_view_instance->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(m_view_instance->m_frameCapturedMutex);

    if (tmpFrame != nullptr || m_off_line) {
        m_view_instance->m_pcFrameAvailable = true;
        m_view_instance->m_abFrameAvailable = true;
        m_view_instance->m_depthFrameAvailable = true;
        m_view_instance->numOfThreads = 3;

        m_view_instance->frameHeight = frameDetails.height;
        m_view_instance->frameWidth = frameDetails.width;

        lock.unlock();

        m_view_instance->m_frameCapturedCv.notify_all();

        if (!m_off_line) {
            m_view_instance->m_ctrl->requestFrame();
        }
        else {
            if (m_offline_change_frame) {
                m_view_instance->m_ctrl->requestFrame();
                m_view_instance->m_ctrl->requestFrameOffline(m_off_line_frame_index);
                m_offline_change_frame = false;
            }
        }

        /*********************************/
        std::unique_lock<std::mutex> imshow_lock(m_view_instance->m_imshowMutex);
        m_view_instance->m_barrierCv.wait(imshow_lock, [&]() {
            return m_view_instance->m_waitKeyBarrier == m_view_instance->numOfThreads;
            });
        m_view_instance->m_waitKeyBarrier = 0;
        /*********************************/

        if (!m_base_file_name.empty()) {
            aditof::FrameHandler fh;
            aditof::Frame* frame = m_view_instance->m_capturedFrame.get();
            fh.SnapShotFrames(m_base_file_name.c_str(), frame, m_view_instance->ab_video_data_8bit, m_view_instance->depth_video_data_8bit);
            if (m_offline_save_all_frames) {
                m_offline_change_frame = true;
                if (SaveAllFramesUpdate()) {
                    m_base_file_name = "";
                }
            }
            else {
                m_base_file_name = "";
            }
        }
    }

    return 0;
}

bool ADIMainWindow::SaveAllFramesUpdate() {
    static std::atomic<int> save_counter(0);
    if (m_off_line && m_offline_save_all_frames) {
        uint32_t max_frame_count;
        GetActiveCamera()->getSensor()->getFrameCount(max_frame_count);
        if (m_off_line_frame_index < max_frame_count) {
            // FIXME: This is incorrect.
            save_counter++;
            if (save_counter > 4) {
                m_off_line_frame_index++;
                save_counter = 0;
            }
            return false;
        }
        else {
            m_offline_save_all_frames = false;
            m_off_line_frame_index = 0;
            save_counter = 0;
            return true;
        }
    }
    return true; // Should never get here since this use case should never occur
}

void ADIMainWindow::DepthLinePlot(ImGuiWindowFlags overlayFlags) {

	if (m_depth_line_values.size() == 0) {
		return;
	}

    SetWindowPosition(m_dict_win_position["plotA"].x, m_dict_win_position["plotA"].y);
    SetWindowSize(m_dict_win_position["plotA"].width, m_dict_win_position["plotA"].height);

    if (ImGui::Begin("Depth Pixel Plot", nullptr, overlayFlags | ImGuiWindowFlags_NoTitleBar)) {
        ImPlot::SetNextAxesToFit();
        if (ImPlot::BeginPlot("Line Depth Values")) {

            std::vector<float> x(m_depth_line_values.size());

            std::iota(x.begin(), x.end(), 0);
            ImPlot::PlotLine("Depth over Line", x.data(), m_depth_line_values.data(), m_depth_line_values.size());
            ImPlot::EndPlot();
        }
        ImGui::End();
    }
}

static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs) {
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}

static inline ImVec2 operator/(const ImVec2& lhs, const float rhs) {
    return ImVec2(lhs.x / rhs, lhs.y / rhs);
}

static inline ImVec2 operator*(const ImVec2& lhs, const float rhs) {
    return ImVec2(lhs.x * rhs, lhs.y * rhs);
}

ImVec2 ADIMainWindow::ImRotate(const ImVec2& v, float cos_a, float sin_a) {
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}

void ADIMainWindow::ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size,
    float angle) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

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
            ImRotate(ImVec2(-_size.x * 0.5f, +_size.y * 0.5f), cos_a, sin_a) };
    ImVec2 uvs[4] = { ImVec2(0.0f, 0.0f), ImVec2(1.0f, 0.0f), ImVec2(1.0f, 1.0f),
                     ImVec2(0.0f, 1.0f) };

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0],
        uvs[1], uvs[2], uvs[3], IM_COL32_WHITE);
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
    m_depth_line_values.clear();
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

        if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            ImVec2 mousePos = ImGui::GetMousePos();
            if (m_depthLine.size() >= 2) {
                m_depthLine.clear();
                depth_line_points.clear();
            } else {
                m_depthLine.push_back({ mousePos.x, mousePos.y });
            }
        }

		if (m_depthLine.size() == 1) {
            ImVec2 mousePos = ImGui::GetMousePos();
			ImVec2 start = ImVec2(m_depthLine[0].first, m_depthLine[0].second);
			ImVec2 end = ImVec2(mousePos.x, mousePos.y);
			ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 2.0f);
		} else if (m_depthLine.size() >= 2) {
            ImVec2 start;
            ImVec2 end;

            if (m_depthLine[0].first < m_depthLine[1].first) {
                start = ImVec2(m_depthLine[0].first, m_depthLine[0].second);
                end = ImVec2(m_depthLine[1].first, m_depthLine[1].second);
            }
            else {
                end = ImVec2(m_depthLine[0].first, m_depthLine[0].second);
                start = ImVec2(m_depthLine[1].first, m_depthLine[1].second);
            }

            ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 2.0f);

            ImVec2 _start = m_invalid_hovered_pixel;
			ImVec2 _end = m_invalid_hovered_pixel; 

            GetHoveredImagePix(_start, ImGui::GetCursorScreenPos(),
                start, m_display_depth_dimensions, m_source_depth_image_dimensions);

            GetHoveredImagePix(_end, ImGui::GetCursorScreenPos(),
                end, m_display_depth_dimensions, m_source_depth_image_dimensions);

            depth_line_points = GetLinePixels(_start.x, _start.y, _end.x, _end.y);

            m_depth_line_values.clear();
			for (const auto& point : depth_line_points) {
                uint32_t offset = m_view_instance->frameWidth * point.y + point.x;
                float depth = m_view_instance->depth_video_data[offset];

                if (depth == 0 && offset > 0)
                    depth = m_view_instance->depth_video_data[offset - 1];

                m_depth_line_values.emplace_back(depth);
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

void ADIMainWindow::InitOpenGLPointCloudTexture() {
    glEnable(GL_PROGRAM_POINT_SIZE);

    constexpr char const pointCloudVertexShader[] =
        R"(
				#version 330 core
				layout (location = 0) in vec3 aPos;
				layout (location = 1) in vec3 hsvColor;//Contains R, G, B, values based on HSV standard

				uniform mat4 model;
				uniform mat4 view;
				uniform mat4 projection;
                uniform float uPointSize;

				out vec4 color_based_on_position;

				void main()
				{
                    vec3 flippedPos = aPos;
                    flippedPos.x = -flippedPos.x; // Flip horizontally
                    if (length(flippedPos) < 0.0001) {
                        gl_PointSize = 10.0;
                        color_based_on_position = vec4(1.0, 1.0, 1.0, 1.0);
                    } else {
                        gl_PointSize = uPointSize;
                        color_based_on_position = vec4(hsvColor, 1.0);
                    }
					gl_Position = projection * view * model * vec4(flippedPos, 1.0);
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
    m_view_instance->projectionIndex = glGetUniformLocation(m_view_instance->pcShader.Id(), "projection");
    m_view_instance->m_pointSizeIndex = glGetUniformLocation(m_view_instance->pcShader.Id(), "uPointSize");

    //Initialize Model, View, and Projection Matrices to Identity
    mat4x4_identity(m_view_mat);
    mat4x4_identity(m_projection_mat);
    mat4x4_identity(m_model_mat);

    //Create Frame Buffers to be able to display on the Point Cloud Window.
    glGenFramebuffers(1, &m_gl_pc_colourTex);
    glBindFramebuffer(GL_FRAMEBUFFER, m_gl_pc_colourTex);
    // create a color attachment texture
    glGenTextures(1, &m_gl_pointcloud_video_texture);
    glBindTexture(GL_TEXTURE_2D, m_gl_pointcloud_video_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_main_window_width, m_main_window_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_gl_pointcloud_video_texture, 0);

    glGenTextures(1, &m_gl_pc_depthTex);
    glBindTexture(GL_TEXTURE_2D, m_gl_pc_depthTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_main_window_width, m_main_window_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_gl_pc_depthTex, 0);

    GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, drawBuffers);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "FBO incomplete!\n";
        return;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    PointCloudReset();
}

void ADIMainWindow::DisplayPointCloudWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    auto imageScale =
        DisplayFrameWindow(ImVec2(m_xyz_position->width, m_xyz_position->height),
                           m_display_point_cloud_dimensions, size);

    SetWindowPosition(m_xyz_position->x, m_xyz_position->y);
    SetWindowSize(m_xyz_position->width, m_xyz_position->height);


    if (ImGui::Begin("Point Cloud Window", nullptr, overlayFlags)) {

        ImGui::SetCursorPos(ImVec2(0, 0));

        ProcessInputs(window);

        if (PreparePointCloudVertices(m_view_instance->vertexBufferObject,
            m_view_instance->vertexArrayObject) >= 0) {

            glBindFramebuffer(GL_FRAMEBUFFER, m_gl_pc_colourTex);
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear whole main window
            glEnable(GL_DEPTH_TEST);

            // draw our Image
            glUseProgram(m_view_instance->pcShader.Id());

            glUniform1f(m_view_instance->m_pointSizeIndex, m_point_size);
            mat4x4_perspective(m_projection_mat, Radians(m_field_of_view), (float)m_view_instance->frameWidth / (float)m_view_instance->frameHeight, 0.1f, 100.0f);
            glUniformMatrix4fv(m_view_instance->projectionIndex, 1, GL_FALSE, &m_projection_mat[0][0]);

            //Look-At function[ x, y, z] = (m_camera_position_vec, m_camera_position_vec + m_camera_front_vec, m_camera_up_vec);
            vec3_add(m_camera_position_front_vec, m_camera_position_vec, m_camera_front_vec);
            mat4x4_look_at(m_view_mat, m_camera_position_vec, m_camera_position_front_vec, m_camera_up_vec);
            glUniformMatrix4fv(m_view_instance->viewIndex, 1, GL_FALSE, &m_view_mat[0][0]);
            glUniformMatrix4fv(m_view_instance->modelIndex, 1, GL_FALSE, &m_model_mat[0][0]);

            glBindVertexArray(m_view_instance->vertexArrayObject); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_view_instance->vertexArraySize));
            glBindVertexArray(0);

            glUseProgram(0);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            ImVec2 _displayPointCloudDimensions = m_display_point_cloud_dimensions;

            if (rotationangledegrees == 90 || rotationangledegrees == 270) {
                std::swap(_displayPointCloudDimensions.x,
                    _displayPointCloudDimensions.y);
            }

            ImageRotated((ImTextureID)m_gl_pointcloud_video_texture,
                ImVec2(m_xyz_position->width, m_xyz_position->height),
                ImVec2(_displayPointCloudDimensions.x,
                    _displayPointCloudDimensions.y),
                rotationangleradians);
            glDeleteVertexArrays(1, &m_view_instance->vertexArrayObject);
            glDeleteBuffers(1, &m_view_instance->vertexBufferObject);

            float modelMatrix[16];
            float projMatrix[16];

            glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 1000.0f);
            memcpy(projMatrix, glm::value_ptr(proj), sizeof(float) * 16);

            memcpy(modelMatrix, m_model_mat, sizeof(float) * 16);
            ImOGuizmo::SetRect(m_xyz_position->x + 5.0f, m_xyz_position->y + 15.0f, 50.0f);
            ImOGuizmo::DrawGizmo(modelMatrix, projMatrix, 10.0f);
            glDisable(GL_DEPTH_TEST);
        }
    }
    ImGui::End();
   
}

int32_t ADIMainWindow::PreparePointCloudVertices(GLuint &vbo, GLuint&vao) {

	if (m_view_instance->normalized_vertices == nullptr) {
		return -1;
	}

    if (m_view_instance->vertexArraySize == 0) {
        return -2;
    }
    
    glGenVertexArrays(1, &vao); // Here is where m_view_instance->vertexArraySize goes to 0;
    //Initialize Point Cloud Image
    glGenBuffers(1, &vbo);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(vao);
    //Bind Point Cloud Image
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //Pass on Image buffer here:
    glBufferData(GL_ARRAY_BUFFER, 
                 m_view_instance->vertexArraySize,
                 m_view_instance->normalized_vertices,
                 GL_STREAM_DRAW);

    assert(m_view_instance->vertexArraySize != 0);

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

    return 0;
}

void ADIMainWindow::PointCloudReset() {

    const mat4x4 m_view_default =
    { {1.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f},
      {-0.0213157870f, -0.00631578919f, -3.0f, 1.0f} };

    const mat4x4 m_projection_default = 
    { {9.51436424f, 0.00000000f, 0.00000000f, 0.00000000f},
      {0.00000000f, 9.51436424f, 0.00000000f, 0.00000000f},
      {0.00000000f, 0.00000000f, -1.00200200f, -1.00000000f},
      {0.00000000f, 0.00000000f, -0.200200200f, 0.00000000f} };

    const mat4x4 m_model_default =
    { {-0.989992976f, 0.0140884947f, -0.140415087f, 0.00000000f},
      {0.00000000f, 0.995004535f, 0.0998334810f, 0.00000000f},
      {0.141119987f, 0.0988343805f, -0.985047400f, 0.00000000f},
      {0.00000000f, 0.00000000f, 0.00000000f, 1.00000000f} };

    memcpy(m_view_mat, m_view_default, sizeof(m_view_mat));
    memcpy(m_projection_mat, m_projection_default, sizeof(m_projection_mat));
    memcpy(m_model_mat, m_model_default, sizeof(m_model_mat));
    
    m_delta_time = 0.1;
    m_field_of_view = 12.0f;
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

void ADIMainWindow::GetYawPitchRoll(float& yaw, float& pitch, float& roll) {
    // Extract rotation matrix (upper-left 3x3)
    float r00 = m_model_mat[0][0], r01 = m_model_mat[0][1], r02 = m_model_mat[0][2];
    float r10 = m_model_mat[1][0], r11 = m_model_mat[1][1], r12 = m_model_mat[1][2];
    float r20 = m_model_mat[2][0], r21 = m_model_mat[2][1], r22 = m_model_mat[2][2];

    // Assuming rotation order is yaw (Y), pitch (X), roll (Z):
    // Yaw (around Y): atan2(r02, r22)
    // Pitch (around X): asin(-r12)
    // Roll (around Z): atan2(r10, r11)

    pitch = std::asin(-r12);
    yaw = std::atan2(r02, r22);
    roll = std::atan2(r10, r11);

    // Convert to degrees
    const float rad2deg = 180.0f / 3.14159265358979323846f;
    yaw *= rad2deg;
    pitch *= rad2deg;
    roll *= rad2deg;
}


void ADIMainWindow::ProcessInputs(GLFWwindow *window) {

    ImGuiIO& io = ImGui::GetIO(); //Get mouse events
    //Sensitivity
    const float maxFov = 45.0f;
    float cameraSpeed = 2.5f * m_delta_time;
    float dRoll = 0.0f;
    float dPitch = 0.0f;
    float dYaw = 0.0f;
    bool update = false;
    
    if (ImGui::IsWindowHovered()) {
        if (io.MouseWheel) {
            if (ImGui::GetIO().KeyAlt && ImGui::GetIO().KeyCtrl) {

                dRoll = -(float)io.MouseWheel / 10.0f;
                update = true;

            } else if (ImGui::GetIO().KeyCtrl) {

                dPitch = -(float)io.MouseWheel / 10.0f;
                update = true;

            } else if (ImGui::GetIO().KeyAlt) {

                dYaw = -(float)io.MouseWheel / 10.0f;
                update = true;

            } else {

                m_field_of_view -= (float)io.MouseWheel;
                if (m_field_of_view < 1.0f) {
                    m_field_of_view = 1.0f;
                }
                if (m_field_of_view > maxFov) {
                    m_field_of_view = maxFov;
                }
            }

            if (update) {
                glm::mat4 incr =
                    glm::rotate(glm::mat4(1.0f), dYaw, glm::vec3(0, 1, 0))   // Yaw (Y axis)
                    * glm::rotate(glm::mat4(1.0f), dPitch, glm::vec3(1, 0, 0))   // Pitch (X axis)
                    * glm::rotate(glm::mat4(1.0f), dRoll, glm::vec3(0, 0, 1));  // Roll (Z axis)

                mat4x4 incr_mat4x4;

                for (int col = 0; col < 4; ++col)
                    for (int row = 0; row < 4; ++row)
                        incr_mat4x4[col][row] = incr[col][row];

                MatrixMultiply(m_model_mat, incr_mat4x4, m_model_mat);

                return;
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