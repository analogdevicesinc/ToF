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

void ADIMainWindow::DisplayInfoWindow(ImGuiWindowFlags overlayFlags) {
    using namespace aditof;
    static bool show_ini_window = false;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    auto camera = getActiveCamera();
    if (!camera) {
        LOG(ERROR) << "No camera found";
        return;
    }

    auto frame = view->m_capturedFrame;
    if (!frame) {
        LOG(ERROR) << "No frame received";
        return;
    }

    m_fps_frameRecvd++;

    CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    uint8_t camera_mode = cameraDetails.mode;

    if (setTempWinPositionOnce) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        setTempWinPositionOnce = false;
    }

    setWindowPosition(dictWinPosition["info"].x, dictWinPosition["info"].y);
    setWindowSize(dictWinPosition["info"].width, dictWinPosition["info"].height);

    if (ImGui::Begin("Information Window", nullptr,
                     overlayFlags)) {

        std::string formattedIP;
        for (int i = 0; i < m_cameraIp.length(); i++)
            formattedIP += toupper(m_cameraIp[i]);

        const char *col1Text = "Frames Received";
        const float padding = 20.0f; // Optional extra space
        float col1Width = ImGui::CalcTextSize(col1Text).x + padding;

        if (ImGui::BeginTable("Information Table", 2)) {
            ImGui::TableSetupColumn(
                "Type", ImGuiTableColumnFlags_WidthFixed, col1Width);
            ImGui::TableSetupColumn("Value",
                                    ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Camera");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(formattedIP.c_str());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Mode");
            ImGui::TableSetColumnIndex(1);
            std::string s = m_cameraModesLookup[static_cast<uint16_t>(camera_mode)]; 
            ImGui::Text(s.c_str());

            if (m_fps_expectedFPS) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Expected fps");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%i", m_fps_expectedFPS);
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Current fps");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%i", m_fps);

            if (camera_mode != 4) { // 4 - pcm-native
                Metadata metadata;
                Status status = frame->getMetadataStruct(metadata);
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to get frame metadata.";
                } else {
                    uint32_t frameNum = metadata.frameNumber;
                    if (m_fps_firstFrame == 0) {
                        m_fps_firstFrame = frameNum;
                    }
                    int32_t sensorTemp = (metadata.sensorTemperature);
                    int32_t laserTemp = (metadata.laserTemperature);
                    uint32_t totalFrames = frameNum - m_fps_firstFrame + 1;
                    uint32_t framesLost = totalFrames - m_fps_frameRecvd;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Frames Received");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%i", m_fps_frameRecvd);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Frames Lost");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%i", framesLost);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Laser Temp");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%i C", laserTemp);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Sensor Temp");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%i C", sensorTemp);
                }
            }

            ImGui::EndTable();
        }
    }

    ImGui::End();
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