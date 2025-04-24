#include "ADIImGUIExtensions.h"
#include "ADIMainWindow.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

inline ImVec2 operator+(const ImVec2 &a, const ImVec2 &b) {
    return ImVec2(a.x + b.x, a.y + b.y);
}
inline ImVec2 operator*(const ImVec2 &v, float f) {
    return ImVec2(v.x * f, v.y * f);
}

using namespace adiMainWindow;

#include <cstdarg> // for va_list
#include <imgui.h>

auto DrawIconButton =
    [](const char *id,
       std::function<void(ImDrawList *, ImVec2, ImVec2)> drawIcon) -> bool {
    ImVec2 size(40, 40);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList *drawList = ImGui::GetWindowDrawList();

    bool pressed = ImGui::InvisibleButton(id, size);
    ImVec2 min = pos;
    ImVec2 max = ImVec2(pos.x + size.x, pos.y + size.y);

    // Draw background if hovered or held
    ImU32 bgColor = IM_COL32(60, 60, 60, 255);
    if (ImGui::IsItemHovered())
        bgColor = IM_COL32(100, 100, 100, 255);
    if (ImGui::IsItemActive())
        bgColor = IM_COL32(150, 150, 150, 255);
    drawList->AddRectFilled(min, max, bgColor, 4.0f);

    drawIcon(drawList, min, max);
    return pressed;
};

void ADIMainWindow::DisplayControlWindow(ImGuiWindowFlags overlayFlags) {
    using namespace aditof;
    static bool show_ini_window = false;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    if (setTempWinPositionOnce) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        setTempWinPositionOnce = false;
    }

    setWindowPosition(dictWinPosition["control"].x, dictWinPosition["control"].y);
    setWindowSize(dictWinPosition["control"].width, dictWinPosition["control"].height);

    if (ImGui::Begin("Control Window", nullptr,
                     overlayFlags)) {

        if (!m_focusedOnce) {
            ImGui::SetWindowFocus();
            m_focusedOnce = true;
        }

        if (DrawIconButton("Play", [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                ImVec2 center = (min + max) * 0.5f;
                float w = max.x - min.x, h = max.y - min.y;
                ImVec2 p1(center.x - w * 0.2f, center.y - h * 0.3f);
                ImVec2 p2(center.x - w * 0.2f, center.y + h * 0.3f);
                ImVec2 p3(center.x + w * 0.3f, center.y);
                dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);
            })) {
            // TODO
        }

        ImGui::SameLine(0.0f, 10.0f);

        if (DrawIconButton("Stop", [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                ImVec2 center = (min + max) * 0.5f;
                float side = (max.x - min.x) * 0.4f;
                ImVec2 pMin(center.x - side * 0.5f, center.y - side * 0.5f);
                ImVec2 pMax(center.x + side * 0.5f, center.y + side * 0.5f);
                dl->AddRectFilled(pMin, pMax, IM_COL32_WHITE);
            })) {
            isPlaying = false;
            isPlayRecorded = false;
            m_fps_frameRecvd = 0;
            CameraStop();
            if (isRecording) {
                view->m_ctrl->stopRecording();
                isRecording = false;
            }
        }

        ImGui::SameLine(0.0f, 10.0f);

        if (DrawIconButton("Pause", [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                ImVec2 center = (min + max) * 0.5f;
                float w = max.x - min.x;
                float h = max.y - min.y;
                float barWidth = w * 0.1f;
                float barHeight = h * 0.5f;
                float spacing = w * 0.1f;

                ImVec2 bar1Min(center.x - spacing - barWidth,
                               center.y - barHeight * 0.5f);
                ImVec2 bar1Max(center.x - spacing, center.y + barHeight * 0.5f);
                ImVec2 bar2Min(center.x + spacing, center.y - barHeight * 0.5f);
                ImVec2 bar2Max(center.x + spacing + barWidth,
                               center.y + barHeight * 0.5f);

                dl->AddRectFilled(bar1Min, bar1Max, IM_COL32_WHITE);
                dl->AddRectFilled(bar2Min, bar2Max, IM_COL32_WHITE);
            })) {

            // TODO
        }

        NewLine(5.0f);

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

        static uint32_t selected;

        ImGui::RadioButton("Depth Colour", selected == 0);
        if (ImGui::IsItemClicked())
            selected = 0;
        ImGui::RadioButton("AB Colour", selected == 1);
        if (ImGui::IsItemClicked())
            selected = 1;

        view->setPointCloudColour(selected);

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