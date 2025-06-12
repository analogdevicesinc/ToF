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

void ADIMainWindow::DisplayInfoWindow(ImGuiWindowFlags overlayFlags, bool diverging) {
    using namespace aditof;
    static bool show_ini_window = false;

    if ((float)m_view_instance->frameWidth == 0.0 &&
        (float)(m_view_instance->frameHeight == 0.0)) {
        return;
    }

    auto camera = GetActiveCamera();
    if (!camera) {
        LOG(ERROR) << "No camera found";
        return;
    }

    auto frame = m_view_instance->m_capturedFrame;
    if (!frame) {
        LOG(ERROR) << "No frame received";
        return;
    }

    CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    uint8_t camera_mode = cameraDetails.mode;

    if (m_set_temp_win_position_once) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        m_set_temp_win_position_once = false;
    }

    SetWindowPosition(m_dict_win_position["info"].x, m_dict_win_position["info"].y);
    SetWindowSize(m_dict_win_position["info"].width, m_dict_win_position["info"].height);

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
            if (m_off_line) {
                ImGui::Text("Offline");
            } else {
                ImGui::Text(formattedIP.c_str());
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Preview Mode");
            ImGui::TableSetColumnIndex(1);
            if (m_enable_preview) {
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255)); // RGBA for red
            }
            else {
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255)); // RGBA for green
            }
            ImGui::Text("%s", m_enable_preview ? "On" : "Off");
            ImGui::PopStyleColor();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Mode");
            ImGui::TableSetColumnIndex(1);
            std::string s = m_cameraModesLookup[static_cast<uint16_t>(camera_mode)]; 
            ImGui::Text(s.c_str());

            if (m_fps_expected) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Expected fps");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%i", m_fps_expected);
            }

            static uint32_t fps;
			m_view_instance->m_ctrl->getFrameRate(fps);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Current fps");
            ImGui::TableSetColumnIndex(1);
            if (m_off_line) {
                ImGui::Text("N/A");
            }
            else {
                ImGui::Text("%i", fps);;
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Frame Rate Warning");
            ImGui::TableSetColumnIndex(1);
            if (diverging) {
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255)); // RGBA for red
            }
            else {
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255)); // RGBA for green
            }
            ImGui::Text("%s", diverging ? "Too High" : "Good");
            ImGui::PopStyleColor();

            if (camera_mode != 4) { // 4 - pcm-native
                Metadata metadata;
                Status status = frame->getMetadataStruct(metadata);
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to get frame metadata.";
                } else {
                    int32_t sensorTemp = (metadata.sensorTemperature);
                    int32_t laserTemp = (metadata.laserTemperature);

                    uint32_t frame_received;
                    m_view_instance->m_ctrl->getFramesReceived(frame_received);
                    uint32_t frames_lost;
                    m_view_instance->m_ctrl->getFramesLost(frames_lost);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Frames Received");
                    ImGui::TableSetColumnIndex(1);
                    if (m_off_line) {
                        ImGui::Text("N/A");
                    } else {
                        ImGui::Text("%i", frame_received);;
                    }

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Frames Lost");
                    ImGui::TableSetColumnIndex(1);
                    if (m_off_line) {
                        ImGui::Text("N/A");
                    }
                    else {
                        ImGui::Text("%i", frames_lost);;
                    }

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

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Point Cloud FoV");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%0.2f", m_field_of_view);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Point Cloud Camera PoS");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("(%0.1f, %0.1f, %0.1f)", m_camera_position_vec[0], m_camera_position_vec[1],
                m_camera_position_vec[2]);

			float roll, pitch, yaw;
			GetYawPitchRoll(yaw, pitch, roll);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Camera (Y, P, R)");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("(%0.1f, %0.1f, %0.1f)", yaw, pitch, roll);

            ImGui::EndTable();
        }
    }

    ImGui::End();
}