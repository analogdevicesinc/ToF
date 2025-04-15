#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

using namespace adiMainWindow;

//*******************************************
//* Section: Generic
//*******************************************

bool ADIMainWindow::DisplayFrameWindow(ImVec2 &displayUpdate, ImVec2 &size) {

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return false;
    }

    size.x = view->frameWidth * imagescale;
    size.y = view->frameHeight * imagescale;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(size.x, size.y);
    }

    displayUpdate = {static_cast<float>(size.x), static_cast<float>(size.y)};

    size.x /= dpiScaleFactor;
    size.y /= dpiScaleFactor;

    return true;
}

void ADIMainWindow::synchronizeDepthABVideo() {

    view->m_capturedFrame = view->m_ctrl->getFrame();

    aditof::FrameDetails frameDetails;
    view->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(view->m_frameCapturedMutex);
    if (displayAB) {
        view->m_abFrameAvailable = true;
    } else {
        view->m_abFrameAvailable = false;
        view->ab_video_data_8bit = nullptr;
    }
    if (displayDepth) {
        view->m_depthFrameAvailable = true;
    } else {
        view->m_depthFrameAvailable = false;
        view->depth_video_data_8bit = nullptr;
    }
    if (displayAB && displayDepth) {
        view->numOfThreads = 2;
    } else {
        view->numOfThreads = 1;
    }

    /************************************/
    //CMOS
    view->frameHeight = frameDetails.height;
    view->frameWidth = frameDetails.width;
    /************************************/

    lock.unlock();
    view->m_frameCapturedCv.notify_all();
    view->m_ctrl->requestFrame();

    /*********************************/
    std::unique_lock<std::mutex> imshow_lock(view->m_imshowMutex);
    view->m_barrierCv.wait(imshow_lock, [&]() {
        return view->m_waitKeyBarrier == view->numOfThreads; /*2*/
        ;
    });
    view->m_waitKeyBarrier = 0;
    /*********************************/
}

void ADIMainWindow::synchronizePointCloudVideo() {
    view->m_capturedFrame = view->m_ctrl->getFrame();

    aditof::FrameDetails frameDetails;
    view->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(view->m_frameCapturedMutex);

    view->m_depthFrameAvailable = true;
    view->m_pointCloudFrameAvailable = true;
    view->frameHeight = frameDetails.height;
    view->frameWidth = frameDetails.width;

    lock.unlock();
    view->m_frameCapturedCv.notify_all();
    view->m_ctrl->requestFrame();

    /*********************************/
    std::unique_lock<std::mutex> imshow_lock(view->m_imshowMutex);
    view->m_barrierCv.wait(imshow_lock, [&]() {
        return view->m_waitKeyBarrier == view->numOfThreads;
    });
    view->m_waitKeyBarrier = 0;
    /*********************************/
}

void ADIMainWindow::GetHoveredImagePix(ImVec2 &hoveredImagePixel,
                                       ImVec2 imageStartPos, ImVec2 mousePos,
                                       ImVec2 displayDimensions) {
    ImVec2 hoveredUIPixel;
    hoveredUIPixel.x = mousePos.x - imageStartPos.x;
    hoveredUIPixel.y = mousePos.y - imageStartPos.y;

    ImVec2 _displayDepthDimensions = displayDepthDimensions;
    ImVec2 _sourceDepthImageDimensions = sourceDepthImageDimensions;

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

    hoveredUIPixel.x = (std::min)(hoveredUIPixel.x, _displayDepthDimensions.x);
    hoveredUIPixel.x = (std::max)(hoveredUIPixel.x, 0.0f);

    hoveredUIPixel.y = (std::min)(hoveredUIPixel.y, _displayDepthDimensions.y);
    hoveredUIPixel.y = (std::max)(hoveredUIPixel.y, 0.0f);

    const float uiCoordinateToImageCoordinateRatio =
        _sourceDepthImageDimensions.x / _displayDepthDimensions.x;

    hoveredImagePixel.x =
        std::round(hoveredUIPixel.x * uiCoordinateToImageCoordinateRatio);
    hoveredImagePixel.y =
        std::round(hoveredUIPixel.y * uiCoordinateToImageCoordinateRatio);

    if (rotationangledegrees == 90) {
        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.y =
            sourceDepthImageDimensions.y - hoveredImagePixel.y;
    } else if (rotationangledegrees == 270) {
        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.x =
            sourceDepthImageDimensions.x - hoveredImagePixel.x;
    } else if (rotationangledegrees == 180) {
        hoveredImagePixel.x =
            _sourceDepthImageDimensions.x - hoveredImagePixel.x;
        hoveredImagePixel.y =
            _sourceDepthImageDimensions.y - hoveredImagePixel.y;
    }
}

//*******************************************
//* Section: Handling of AB Window 
//*******************************************

void ADIMainWindow::initOpenGLABTexture() {
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
    ab_video_texture = ab_texture;
    /*************************************************/
}

void ADIMainWindow::DisplayActiveBrightnessWindow(
    ImGuiWindowFlags overlayFlags) {

    ImVec2 size;

    if (DisplayFrameWindow(displayABDimensions, size) == false)
        return;

    dictWinPosition["ab"] = std::array<float, 4>(
        {offsetfromleft,
         dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
         size.y});
    setWindowPosition(dictWinPosition["ab"][0], dictWinPosition["ab"][1]);
    setWindowSize(dictWinPosition["ab"][2] + 40,
                  dictWinPosition["ab"][3] + 130);

    if (ImGui::Begin("Active Brightness Window", nullptr, overlayFlags)) {

        bool logImage = view->getLogImage();
        bool autoScale = view->getAutoScale();

        ImGui::SameLine();
        ImGui::Checkbox("Auto-scale", &autoScale);

        if (!autoScale && logImage) {
            logImage = false;
        } else if (autoScale) {
            ImGui::SameLine();
            ImGui::Checkbox("Log Image", &logImage);
        }
        view->setLogImage(logImage);
        view->setAutoScale(autoScale);

        tofImagePosY = ImGui::GetCursorPosY();

        CaptureABVideo();
        ImVec2 hoveredImagePixel = InvalidHoveredPixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
                           ImGui::GetIO().MousePos, displayABDimensions);
        RenderInfoPane(hoveredImagePixel, view->ab_video_data, view->frameWidth,
                       ImGui::IsWindowHovered(),
                       ADI_Image_Format_t::ADI_IMAGE_FORMAT_AB16, "mm");
    }

    ImGui::End();
}

//*******************************************
//* Section: Handling of Depth Widow
//*******************************************

void ADIMainWindow::initOpenGLDepthTexture() {
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
    depth_video_texture = depth_texture;
    /*************************************************/
}

void ADIMainWindow::DisplayDepthWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    if (DisplayFrameWindow(displayDepthDimensions, size) == false)
        return;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    sourceDepthImageDimensions = {(float)(view->frameWidth),
                                  (float)(view->frameHeight)};

    if (displayAB)
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["ab"][0] + dictWinPosition["ab"][2],
             dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
             size.y});
    else
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["pc"][0] + dictWinPosition["pc"][2],
             dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
             size.y});
    setWindowPosition(dictWinPosition["depth"][0] + 40,
                      dictWinPosition["depth"][1]);
    setWindowSize(dictWinPosition["depth"][2] + 40,
                  dictWinPosition["depth"][3] + 130);

    std::string title = "Depth Window";
    if (ImGui::Begin(title.c_str(), nullptr, overlayFlags)) {

        if (!m_focusedOnce) {
            ImGui::SetWindowFocus();
            m_focusedOnce = true;
        }

        if (tofImagePosY != -1.0f) {
            ImGui::SetCursorPosY(tofImagePosY);
        } else {
            tofImagePosY = ImGui::GetCursorPosY();
        }

        CaptureDepthVideo();
        ImVec2 hoveredImagePixel = InvalidHoveredPixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
                           ImGui::GetIO().MousePos, displayDepthDimensions);
        RenderInfoPane(hoveredImagePixel, view->depth_video_data,
                       view->frameWidth, ImGui::IsWindowHovered(),
                       ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16, "mm");
    }

    ImGui::End();
}

//*******************************************
//* Section: Handling of Point Cloud Window
//*******************************************

void ADIMainWindow::DisplayPointCloudWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    if (DisplayFrameWindow(displayPointCloudDimensions, size) == false)
        return;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    sourcePointCloudImageDimensions = {(float)(view->frameWidth),
                                       (float)(view->frameHeight)};

    dictWinPosition["pc"] = std::array<float, 4>(
        {offsetfromleft,
         dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
         size.y});

    setWindowPosition(dictWinPosition["pc"][0], dictWinPosition["pc"][1]);
    setWindowSize(dictWinPosition["pc"][2] + 40,
                  dictWinPosition["pc"][3] + 130);

    if (ImGui::Begin("Point Cloud Window", nullptr, overlayFlags)) {

        if (tofImagePosY != -1.0f) {
            ImGui::SetCursorPosY(tofImagePosY);
        }

        CapturePointCloudVideo();

        ImGui::SameLine();
        ImGuiExtensions::ADISliderInt("", &pointSize, 1, 10,
                                      "Point Size: %d px");

        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Reset", true)) {
            pointCloudReset();
        }
    }
    ImGui::End();
}

void ADIMainWindow::preparePointCloudVertices(unsigned int &vbo,
                                              unsigned int &vao) {
    glGenVertexArrays(1, &vao);
    //Initialize Point Cloud Image
    glGenBuffers(1, &vbo);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(vao);
    //Bind Point Cloud Image
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //Pass on Image buffer here:
    glBufferData(GL_ARRAY_BUFFER, view->vertexArraySize,
                 view->normalized_vertices,
                 GL_DYNAMIC_DRAW); //GL_STATIC_DRAW

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

void ADIMainWindow::initOpenGLPointCloudTexture() {
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
					//FragColor = vec4(0.1f, 0.8f, 0.1f, 1.0f);
					FragColor = color_based_on_position;
				}
				)";

    //Build and compile our shaders
    adiviewer::ADIShader vertexShader(
        GL_VERTEX_SHADER,
        pointCloudVertexShader); //Our vertices (whole image)
    adiviewer::ADIShader fragmentShader(GL_FRAGMENT_SHADER,
                                        pointCloudFragmentShader); //Color map
    view->pcShader.CreateProgram();
    view->pcShader.AttachShader(std::move(vertexShader));
    view->pcShader.AttachShader(std::move(fragmentShader));
    view->pcShader.Link();

    //Get the model, view, and projection index
    view->modelIndex = glGetUniformLocation(view->pcShader.Id(), "model");
    view->viewIndex = glGetUniformLocation(view->pcShader.Id(), "view");
    view->projectionIndex =
        glGetUniformLocation(view->pcShader.Id(), "projection");

    //Initialize Model, View, and Projection Matrices to Identity
    mat4x4_identity(m_view);
    mat4x4_identity(m_projection);
    mat4x4_identity(m_model);

    //Create Frame Buffers to be able to display on the Point Cloud Window.
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // create a color attachment texture
    glGenTextures(1, &pointCloud_video_texture);
    glBindTexture(GL_TEXTURE_2D, pointCloud_video_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mainWindowWidth, mainWindowHeight, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           pointCloud_video_texture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
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
        ((center * dpiScaleFactor) / 2.0f) + ImGui::GetCursorScreenPos();
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

void ADIMainWindow::CaptureDepthVideo() {
    if (view->depth_video_data_8bit != nullptr) {
        glBindTexture(GL_TEXTURE_2D, depth_video_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, view->frameWidth,
                     view->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     view->depth_video_data_8bit);
        glGenerateMipmap(GL_TEXTURE_2D);
        delete view->depth_video_data_8bit;

        ImVec2 _displayDepthDimensions = displayDepthDimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayDepthDimensions.x, _displayDepthDimensions.y);
        }

        ImageRotated(
            (ImTextureID)depth_video_texture,
            ImVec2(dictWinPosition["depth"][2], dictWinPosition["depth"][3]),
            ImVec2(_displayDepthDimensions.x, _displayDepthDimensions.y),
            rotationangleradians);
    }
}

void ADIMainWindow::CaptureABVideo() {
    if (view->ab_video_data_8bit != nullptr) {
        glBindTexture(GL_TEXTURE_2D, ab_video_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, view->frameWidth,
                     view->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     view->ab_video_data_8bit);
        glGenerateMipmap(GL_TEXTURE_2D);
        delete view->ab_video_data_8bit;

        ImVec2 _displayABDimensions = displayABDimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayABDimensions.x, _displayABDimensions.y);
        }

        ImageRotated((ImTextureID)ab_video_texture,
                     ImVec2(dictWinPosition["ab"][2], dictWinPosition["ab"][3]),
                     ImVec2(_displayABDimensions.x, _displayABDimensions.y),
                     rotationangleradians);
    }
}

void ADIMainWindow::CapturePointCloudVideo() {

    processInputs(window);

    preparePointCloudVertices(view->vertexBufferObject,
                              view->vertexArrayObject);

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPointSize(pointSize);

    // draw our Image
    glUseProgram(view->pcShader.Id());
    mat4x4_perspective(m_projection, radians(fov),
                       (float)view->frameWidth / (float)view->frameHeight, 0.1f,
                       100.0f);
    glUniformMatrix4fv(view->projectionIndex, 1, GL_FALSE, &m_projection[0][0]);

    //Look-At function[ x, y, z] = (cameraPos, cameraPos + cameraFront, cameraUp);
    vec3_add(cameraPos_Front, cameraPos, cameraFront);
    mat4x4_look_at(m_view, cameraPos, cameraPos_Front, cameraUp);
    glUniformMatrix4fv(view->viewIndex, 1, GL_FALSE, &m_view[0][0]);
    glUniformMatrix4fv(view->modelIndex, 1, GL_FALSE, &m_model[0][0]);

    glBindVertexArray(
        view->vertexArrayObject); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
    glDrawArrays(GL_POINTS, 0, view->vertexArraySize);
    glBindVertexArray(0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);

    ImVec2 _displayPointCloudDimensions = displayPointCloudDimensions;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(_displayPointCloudDimensions.x,
                  _displayPointCloudDimensions.y);
    }

    ImageRotated(
        (ImTextureID)pointCloud_video_texture,
        ImVec2(dictWinPosition["pc"][2], dictWinPosition["pc"][3]),
        ImVec2(_displayPointCloudDimensions.x, _displayPointCloudDimensions.y),
        rotationangleradians);
    glDeleteVertexArrays(1, &view->vertexArrayObject);
    glDeleteBuffers(1, &view->vertexBufferObject);
}

void ADIMainWindow::pointCloudReset() {
    mat4x4_identity(m_view);
    mat4x4_identity(m_projection);
    mat4x4_identity(m_model);
    deltaTime = 0;
    lastFrame = 0;
    fov = 8.0f;
    yaw = -90.0f;
    pitch = 0.0f;
    view->Max_X = 6000.0;
    view->Max_Y = 6000.0;
    view->Max_Z = 6000.0;

    cameraPos[0] = 0.0f;
    cameraPos[1] = 0.0f;
    cameraPos[2] = 3.0f;
    cameraFront[0] = 0.0;
    cameraFront[1] = 0.0;
    cameraFront[2] = -1.0;
    cameraUp[0] = 0.0;
    cameraUp[1] = 1.0;
    cameraUp[2] = 0.0;
    pointSize = 1;
}

float ADIMainWindow::radians(float degrees) {
    float radians = (IM_PI / 180) * degrees;
    return radians;
}

void ADIMainWindow::processInputs(GLFWwindow *window) {
    //Sensitivity
    const float maxFov = 45.0f;
    float cameraSpeed = 2.5f * deltaTime;

    ImGuiIO &io = ImGui::GetIO(); //Get mouse events

    if (ImGui::IsWindowHovered()) {
        if (io.MouseWheel) //Use mouse wheel to scroll up/down for zoom in/zoom out
        {
            fov -= (float)io.MouseWheel;
            if (fov < 1.0f) {
                fov = 1.0f;
            }
            if (fov > maxFov) {
                fov = maxFov;
            }
        }
    }

    MouseMovementType movementType = MouseMovementType::None;

    ImVec2 mouseDownPos(-1.f, -1.f);
    //Press the left button for image rotation
    if (io.MouseDown[GLFW_MOUSE_BUTTON_1]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_1];
        movementType = MouseMovementType::Rotation;
        mouseDown = true;
    }
    //Press the right button for image translation
    else if (io.MouseDown[GLFW_MOUSE_BUTTON_2]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_2];
        movementType = MouseMovementType::Translation;
        mouseDown = true;
    } else {
        mouseDown = false;
    }

    if (mouseDown) {
        const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
        // Normalize to the image start coordinates
        //
        mouseDownPos.x -= imageStartPos.x;
        mouseDownPos.y -= imageStartPos.y;

        const vec2 mouseDelta{io.MouseDelta.x, io.MouseDelta.y};
        const vec2 displayDimensions{static_cast<float>(mainWindowWidth),
                                     static_cast<float>(mainWindowHeight)};
        // Only count drags if they originated on the image
        //
        if (mouseDownPos.x >= 0.f &&
            mouseDownPos.x <= displayPointCloudDimensions.x &&
            mouseDownPos.y >= 0.f &&
            mouseDownPos.y <= displayPointCloudDimensions.y) {
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

                MatrixMultiply(m_model, newRotationMtx, m_model);
            }
            if (movementType == MouseMovementType::Translation) {
                cameraPos[0] -=
                    mouseDelta[0] * TranslationSensitivity / (maxFov - fov + 1);
                cameraPos[1] -=
                    mouseDelta[1] * TranslationSensitivity / (maxFov - fov + 1);
            }
        }
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, cameraFront, cameraSpeed);
        vec3_add(cameraPos, cameraPos, aux);
    }

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, cameraFront, cameraSpeed);
        vec3_sub(cameraPos, cameraPos, aux);
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, cameraFront, cameraUp);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_sub(cameraPos, cameraPos, aux);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, cameraFront, cameraUp);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_add(cameraPos, cameraPos, aux);
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