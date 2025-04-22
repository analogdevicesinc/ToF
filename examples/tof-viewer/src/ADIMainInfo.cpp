#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace adiMainWindow;

#include <cstdarg> // for va_list
#include <imgui.h>

void DrawColoredLabel(const char *fmt, ...) {
    // Format the text
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ImVec4 box_color = ImVec4(0.2f, 0.6f, 0.9f, 1.0f); // RGBa

    // Get draw list and cursor position
    ImVec2 textPos = ImGui::GetCursorScreenPos();
    ImVec2 textSize = ImGui::CalcTextSize(buf);
    float padding = 2.0f;

    // Draw background box
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(ImVec2(textPos.x - padding, textPos.y - padding),
                            ImVec2(textPos.x + textSize.x + padding,
                                   textPos.y + textSize.y + padding),
                            ImGui::ColorConvertFloat4ToU32(box_color),
                            4.0f // optional: corner radius
    );

    // Draw the text on top
    ImGui::SetCursorScreenPos(textPos);
    ImGui::TextUnformatted(buf);
}


void ADIMainWindow::DisplayInfoWindow(ImGuiWindowFlags overlayFlags) {
    using namespace aditof;
    static bool show_ini_window = false;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    if (setTempWinPositionOnce) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        imagescale = (_isHighDPI) ? 1.0f : 0.5f;
        setTempWinPositionOnce = false;
    }

    float info_width = 900.0f;
    float info_height = (_isHighDPI) ? 205.0f : 180.0f;

    if (displayAB)
        dictWinPosition["info"] = std::array<float, 4>(
            {offsetfromleft, offsetfromtop, info_width, info_height});
    else
        dictWinPosition["info"] = std::array<float, 4>(
            {offsetfromleft, offsetfromtop, info_width, info_height});

    setWindowPosition(dictWinPosition["info"][0], dictWinPosition["info"][1]);
    setWindowSize(dictWinPosition["info"][2], dictWinPosition["info"][3]);

    if (ImGui::Begin("Information Window", nullptr,
                     overlayFlags | ImGuiWindowFlags_NoTitleBar)) {

        if (!m_focusedOnce) {
            ImGui::SetWindowFocus();
            m_focusedOnce = true;
        }
        std::string formattedIP;
        for (int i = 0; i < m_cameraIp.length(); i++)
            formattedIP += toupper(m_cameraIp[i]);

        ImGui::Text(" Camera ");
        ImGui::SameLine();
        DrawColoredLabel("%s", formattedIP.c_str());

        if (isPlaying || isRecording) {
            auto camera = getActiveCamera();
            if (!camera) {
                LOG(ERROR) << "No camera found";
                return;
            }
            auto frame = view->m_capturedFrame;
            if (!frame) {
                LOG(ERROR) << "No frame received";
                return;
            } else {
                m_fps_frameRecvd++;
            }

            CameraDetails cameraDetails;
            camera->getDetails(cameraDetails);
            uint8_t camera_mode = cameraDetails.mode;

            ImGui::Text(" Camera Mode ");
            ImGui::SameLine();
            DrawColoredLabel("%i", camera_mode);
            if (m_fps_expectedFPS) {
                ImGui::Text(" Frame Rate (Current/Expected) ");
                ImGui::SameLine();
                DrawColoredLabel("%i/%i", m_fps, m_fps_expectedFPS);
            }
            else {
                ImGui::Text(" Current FPS ");
                ImGui::SameLine();
                DrawColoredLabel("%i/%i", m_fps, "Unknown");
            }

            if (camera_mode != 4) { // 4 - pcm-native
                Metadata metadata;
                Status status = frame->getMetadataStruct(metadata);
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to get frame metadata.";
                } else {
                    uint32_t frameNum = (metadata.frameNumber);
                    if (!m_fps_firstFrame) {
                        m_fps_firstFrame = frameNum;
                    }
                    int32_t sensorTemp = (metadata.sensorTemperature);
                    int32_t laserTemp = (metadata.laserTemperature);
                    uint32_t totalFrames = frameNum - m_fps_firstFrame + 1;
                    uint32_t frameLost = totalFrames - m_fps_frameRecvd;
                    ImGui::Text(" Frames Rx, Lost ");
                    ImGui::SameLine();
                    DrawColoredLabel("%i, %i", m_fps_frameRecvd, frameLost);
                    ImGui::Text(" Laser, Sensor Temp ");
                    ImGui::SameLine();
                    DrawColoredLabel("%iC, %iC", laserTemp, sensorTemp);
                }
            }
        }

        //ImGui::NewLine();
        ImGui::Text(" Rotate: ");
        ImGui::SameLine();
        bool rotate = ImGui::Button("+");
        ImGui::SameLine();
        if (rotate) {
            rotationangleradians += M_PI / 2;
            rotationangledegrees += 90;
            if (rotationangledegrees >= 360) {
                rotationangleradians = 0;
                rotationangledegrees = 0;
            }
        }
        ImGui::Text("%i", rotationangledegrees);
        ImGui::Text(" Point Cloud Control: ");
        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Reset", true)) {
            pointCloudReset();
        }
        ImGui::SameLine();
        ImGui::SliderInt("", &pointSize, 1, 10,
                                      "Point Size: %d px");

        // "Stop" button
        ImGuiExtensions::ButtonColorChanger colorChangerStop(customColorStop,
                                                             isPlaying);
        if (isPlaying && !isRecording) {
            ImGui::Text(" View Options:");
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Depth with Active Brightness",
                                            &viewSelection, 0);
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Depth with Point Cloud",
                                            &viewSelection, 1);
            if (ImGuiExtensions::ADIButton("Stop Streaming", isPlaying)) {
                isPlaying = false;
                isPlayRecorded = false;
                m_fps_frameRecvd = 0;
                CameraStop();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
        }
        if (isRecording) {
            ImGui::Text(" View Options:");
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Active Brightness and Depth",
                                            &viewSelection, 0);
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Point Cloud and Depth",
                                            &viewSelection, 1);
            if (ImGuiExtensions::ADIButton("Stop Recording", isPlaying)) {
                isPlaying = false;
                isPlayRecorded = false;
                m_fps_frameRecvd = 0;
                CameraStop();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
        }
        if (isPlayRecorded) {
            ImGui::SameLine();
            if (view != nullptr &&
                view->m_ctrl->m_recorder->getStopPlayback()) {
                stopPlayback();
                view->m_ctrl->m_recorder->setStopPlayback(false);
            }

            { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
                ImGuiExtensions::ButtonColorChanger colorChangerStopPB(
                    customColorStop, isPlayRecorded);
                if (ImGuiExtensions::ADIButton("Close Recording",
                                               isPlayRecorded)) {
                    stopPlayback();
                }
            }

            uint32_t totalFrames =
                view->m_ctrl->m_recorder->getNumberOfFrames();

            rawSeeker = view->m_ctrl->m_recorder->getPlaybackFrameNumber();

            ImGui::SliderInt("Frame #", &rawSeeker, 0,
                                          totalFrames - 1, "%d");

            view->m_ctrl->m_recorder->setPlaybackFrameNumber(rawSeeker);
        }
    }

    ImGui::End();
}

void ADIMainWindow::RenderInfoPane(ImVec2 hoveredImagePixel,
                                   uint16_t *currentImage, int imageWidth,
                                   bool isHovered, ADI_Image_Format_t format,
                                   std::string units) {
    if (static_cast<int>(hoveredImagePixel.x) ==
            static_cast<int>(InvalidHoveredPixel.x) &&
        static_cast<int>(hoveredImagePixel.y) ==
            static_cast<int>(InvalidHoveredPixel.y)) {
        return;
    }

    ImGuiWindowFlags overlayFlags2 =
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar;

    ImGui::SetNextWindowPos(ImGui::GetCursorScreenPos(), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.3f); // Transparent background
    uint16_t pixelValue = 0;

    if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16) {

        ImGui::Begin("dd", nullptr, overlayFlags2);

        if (hoveredImagePixel.x >= 0 && hoveredImagePixel.y >= 0) {
            pixelValue =
                currentImage[((int)hoveredImagePixel.y * (imageWidth)) +
                             int(hoveredImagePixel
                                     .x)]; //153280 is pixel value linear

            if (isHovered || ImGui::IsWindowHovered()) {
                ImGui::Text("Current pixel: %d, %d", int(hoveredImagePixel.x),
                            int(hoveredImagePixel.y));
                ImGui::Text("Current pixel value: %d %s", pixelValue,
                            units.c_str());
                //ImGui::Text("Temperature: %d %s", view->temperature_c, " C");//Not ready to implement yet.
            } else {
                ImGui::Text("Hover over the image to get pixel value");
            }
        }
        ImGui::End();
    } else if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_AB16) {
        ImGui::Begin("dd", nullptr, overlayFlags2);

        if (hoveredImagePixel.x >= 0 && hoveredImagePixel.y >= 0) {
            pixelValue =
                currentImage[((int)hoveredImagePixel.y * (imageWidth)) +
                             int(hoveredImagePixel
                                     .x)]; //153280 is pixel value linear

            if (isHovered || ImGui::IsWindowHovered()) {
                ImGui::Text("Current pixel: %d, %d", int(hoveredImagePixel.x),
                            int(hoveredImagePixel.y));
                ImGui::Text("Current pixel value: %d %s", pixelValue,
                            units.c_str());
            } else {
                ImGui::Text("Hover over the image to get pixel value");
            }
        }
        ImGui::End();
    }
}

void ADIMainWindow::computeFPS(int &fps) {
    frameCounter++;
    auto currentTime = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed = currentTime - m_fps_startTime;
    if (elapsed.count() >= 2) {
        fps = frameCounter / (int)elapsed.count();
        frameCounter = 0;
        m_fps_startTime = currentTime;
    }
}