#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace adiMainWindow;

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
        ImGui::Text(" Camera %s", formattedIP.c_str());
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
        ImGui::SameLine();

        ImGui::Text(" | Scale: ");
        ImGui::SameLine();
        bool scale05 = ImGui::Button("0.5x");
        ImGui::SameLine();
        bool scale10 = ImGui::Button("1x");
        ImGui::SameLine();
        ImGui::PushItemWidth(300 * dpiScaleFactor);
        ImGui::SliderFloat("", &imagescale, 0.25f, 3.0f);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (scale05)
            imagescale = 0.5f;
        if (scale10)
            imagescale = 1.0f;
        ImGui::Text("%ipx x %ipx -> %ipx x %ipx", view->frameWidth,
                    view->frameHeight, (uint32_t)displayABDimensions.x,
                    (uint32_t)displayABDimensions.y);

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

            ImGui::Text(" Camera Mode: %d", camera_mode);
            ImGui::Text(" Current FPS: %i", m_fps);
            if (m_fps_expectedFPS) {
                ImGui::SameLine();
                ImGui::Text(" | Expected FPS: %i", m_fps_expectedFPS);
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
                    ImGui::Text(" Number of frames lost: %u", frameLost);
                    ImGui::SameLine();
                    ImGui::Text(" | Number of frames received: %u",
                                m_fps_frameRecvd);
                    ImGui::Text(" Laser Temperature: %iC", laserTemp);
                    ImGui::SameLine();
                    ImGui::Text(" | Sensor Temperature: %iC", sensorTemp);
                }
            }
        }

        ImGui::NewLine();

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

            ImGuiExtensions::ADISliderInt("Frame #", &rawSeeker, 0,
                                          totalFrames - 1, "%d", true);

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