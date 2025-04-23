#include "ADIImGUIExtensions.h"
#include "ADIMainWindow.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

using namespace adiMainWindow;

#include <cstdarg> // for va_list
#include <imgui.h>

void ADIMainWindow::DisplayControlWindow(ImGuiWindowFlags overlayFlags) {
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

    setWindowPosition(dictWinPosition["control"].x, dictWinPosition["control"].y);
    setWindowSize(dictWinPosition["control"].width, dictWinPosition["control"].height);

    if (ImGui::Begin("Control Window", nullptr,
                     overlayFlags | ImGuiWindowFlags_NoTitleBar)) {

        if (!m_focusedOnce) {
            ImGui::SetWindowFocus();
            m_focusedOnce = true;
        }

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

        DrawBarLabel("Rotate");
        NewLine(5.0f);
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
        NewLine(5.0f);

        DrawBarLabel("Point Cloud");
        NewLine(5.0f);
        if (ImGuiExtensions::ADIButton("Reset", true)) {
            pointCloudReset();
        }
        NewLine(5.0f);
        ImGui::SliderInt("", &pointSize, 1, 10, "Point Size: %d px");

        NewLine(5.0f);

        DrawBarLabel("Active Brightness");
        NewLine(5.0f);
        bool logImage = view->getLogImage();
        bool autoScale = view->getAutoScale();

        ImGui::Checkbox("Auto-scale", &autoScale);
        if (autoScale == false) {
            logImage = false;
        } 

        NewLine(5.0f);
        if (autoScale == false) {
            ImGui::BeginDisabled();
        }
        ImGui::Checkbox("Log Image", &logImage);
        if (autoScale == false) {
            ImGui::EndDisabled();
        }

        view->setLogImage(logImage);
        view->setAutoScale(autoScale);
        NewLine(5.0f);
    }

    ImGui::End();
}