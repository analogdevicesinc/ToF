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

bool DrawIconButton(const char *id,
                    std::function<void(ImDrawList *, ImVec2, ImVec2)> drawIcon,
                    ImU32 bgColor = IM_COL32(60, 60, 60, 255)) {
    ImVec2 size(30, 30);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList *drawList = ImGui::GetWindowDrawList();

    bool pressed = ImGui::InvisibleButton(id, size);
    ImVec2 min = pos;
    ImVec2 max = ImVec2(pos.x + size.x, pos.y + size.y);

    if (ImGui::IsItemActive())
        bgColor = IM_COL32(180, 0, 0, 255);
    else if (ImGui::IsItemHovered())
        bgColor = IM_COL32(255, 165, 0, 255);

    drawList->AddRectFilled(min, max, bgColor, 4.0f);

    drawIcon(drawList, min, max);
    return pressed;
}

void ADIMainWindow::DisplayControlWindow(ImGuiWindowFlags overlayFlags) {
    using namespace aditof;
    static bool show_ini_window = false;

    if ((float)m_view_instance->frameWidth == 0.0 &&
        (float)(m_view_instance->frameHeight == 0.0)) {
        return;
    }

    if (m_set_temp_win_position_once) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        m_set_temp_win_position_once = false;
    }

    SetWindowPosition(m_dict_win_position["control"].x, m_dict_win_position["control"].y);
    SetWindowSize(m_dict_win_position["control"].width, m_dict_win_position["control"].height);

    if (ImGui::Begin("Control Window", nullptr, overlayFlags)) {

        if (!m_focused_once) {
            ImGui::SetWindowFocus();
            m_focused_once = true;
        }

        if (!m_off_line) {

            static std::string filePath = "";
            static bool recordingActive = false;
            if (DrawIconButton(
                    "Record",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center =
                            ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f);
                        float radius = (max.x - min.x) * 0.2f;

                        // Draw a white circle in the center for the record icon
                        dl->AddCircleFilled(center, radius, IM_COL32_WHITE);
                    },
                    (!recordingActive) ? IM_COL32(200, 0, 0, 255)
                                       : IM_COL32(0, 200, 0, 255))) {

                if (!recordingActive) {
                    aditof::Status status =
                        GetActiveCamera()->startRecording(filePath);
                    if (status == aditof::Status::OK) {
                        recordingActive = true;
                        LOG(INFO) << "Recording to " << filePath.c_str();
                    } else {
                        LOG(INFO) << "Unable to start recording.";
                        filePath = "";
                        recordingActive = false;
                    }
                } else {
                    aditof::Status status = GetActiveCamera()->stopRecording();
                    LOG(INFO) << "Recording stopped.";
                    filePath = "";
                    recordingActive = false;
                }
            }

            ImGui::SameLine(0.0f, 10.0f);

            if (DrawIconButton("Stop", [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                    ImVec2 center = (min + max) * 0.5f;
                    float side = (max.x - min.x) * 0.4f;
                    ImVec2 pMin(center.x - side * 0.5f, center.y - side * 0.5f);
                    ImVec2 pMax(center.x + side * 0.5f, center.y + side * 0.5f);
                    dl->AddRectFilled(pMin, pMax, IM_COL32_WHITE);
                })) {
                m_is_playing = false;
                m_fps_frame_received = 0;
                filePath = "";
                CameraStop();
            }

            ImGui::SameLine(0.0f, 10.0f);

        }  else {

            uint32_t max_frame_count;
            GetActiveCamera()->getSensor()->getFrameCount(max_frame_count);

            if (DrawIconButton(
                    "JumpToStart",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center = (min + max) * 0.5f;
                        float w = max.x - min.x;
                        float h = max.y - min.y;

                        float arrowW = w * 0.2f;
                        float arrowH = h * 0.35f;
                        float spacing = w * 0.05f;

                        // Triangle
                        ImVec2 p1(center.x + spacing + arrowW * 0.5f,
                                  center.y - arrowH);
                        ImVec2 p2(center.x + spacing + arrowW * 0.5f,
                                  center.y + arrowH);
                        ImVec2 p3(center.x + spacing - arrowW * 0.5f, center.y);

                        dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);

                        // Bar (vertical line to the left of triangle)
                        float barX = center.x - spacing - arrowW * 0.5f;
                        dl->AddRectFilled(
                            ImVec2(barX - 1.0f, center.y - arrowH),
                            ImVec2(barX + 1.0f, center.y + arrowH),
                            IM_COL32_WHITE);
                    },
                    IM_COL32(60, 60, 60, 255))) {

                m_off_line_frame_index = 0;

            }

            ImGui::SameLine(0.0f, 10.0f);

            if (DrawIconButton(
                    "LeftArrow",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center = (min + max) * 0.5f;
                        float w = max.x - min.x;
                        float h = max.y - min.y;

                        float arrowW = w * 0.25f;
                        float arrowH = h * 0.3f;

                        ImVec2 p1(center.x + arrowW * 0.5f,
                                  center.y - arrowH); // tip top
                        ImVec2 p2(center.x + arrowW * 0.5f,
                                  center.y + arrowH); // tip bottom
                        ImVec2 p3(center.x - arrowW * 0.5f,
                                  center.y); // back center

                        dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);
                    },
                    IM_COL32(50, 50, 50, 255))) {

                if (m_off_line_frame_index > 0) {
                    m_off_line_frame_index--;
                }
            }

            ImGui::SameLine(0.0f, 10.0f);
#if 0
            if (DrawIconButton(
                     "Play", [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                ImVec2 center = (min + max) * 0.5f;
                float w = max.x - min.x, h = max.y - min.y;
                ImVec2 p1(center.x - w * 0.2f, center.y - h * 0.3f);
                ImVec2 p2(center.x - w * 0.2f, center.y + h * 0.3f);
                ImVec2 p3(center.x + w * 0.3f, center.y);
                dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);
            })) {

                // TODO: Play button

            }

            ImGui::SameLine(0.0f, 10.0f);

            if (DrawIconButton(
                    "Pause",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center = (min + max) * 0.5f;
                        float w = max.x - min.x;
                        float h = max.y - min.y;
                        float barWidth = w * 0.1f;
                        float barHeight = h * 0.5f;
                        float spacing = w * 0.1f;

                        ImVec2 bar1Min(center.x - spacing - barWidth,
                                       center.y - barHeight * 0.5f);
                        ImVec2 bar1Max(center.x - spacing,
                                       center.y + barHeight * 0.5f);
                        ImVec2 bar2Min(center.x + spacing,
                                       center.y - barHeight * 0.5f);
                        ImVec2 bar2Max(center.x + spacing + barWidth,
                                       center.y + barHeight * 0.5f);

                        dl->AddRectFilled(bar1Min, bar1Max, IM_COL32_WHITE);
                        dl->AddRectFilled(bar2Min, bar2Max, IM_COL32_WHITE);
                    },
                    IM_COL32(60, 60, 60, 255))) {

                // TODO
            }
#endif //0 

            ImGui::SameLine(0.0f, 10.0f);
            
            if (DrawIconButton(
                    "RightArrow",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center = (min + max) * 0.5f;
                        float w = max.x - min.x;
                        float h = max.y - min.y;

                        float arrowW = w * 0.25f;
                        float arrowH = h * 0.3f;

                        ImVec2 p1(center.x - arrowW * 0.5f,
                                  center.y - arrowH); // tip top
                        ImVec2 p2(center.x - arrowW * 0.5f,
                                  center.y + arrowH); // tip bottom
                        ImVec2 p3(center.x + arrowW * 0.5f,
                                  center.y); // back center

                        dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);
                    },
                    IM_COL32(50, 50, 50, 255))) {

                if (m_off_line_frame_index < max_frame_count) { // TODO: FIXME
                    m_off_line_frame_index++;
                }
            }

            ImGui::SameLine(0.0f, 10.0f);

            if (DrawIconButton(
                    "JumpToEnd",
                    [](ImDrawList *dl, ImVec2 min, ImVec2 max) {
                        ImVec2 center = (min + max) * 0.5f;
                        float w = max.x - min.x;
                        float h = max.y - min.y;

                        float arrowW = w * 0.2f;
                        float arrowH = h * 0.35f;
                        float spacing = w * 0.05f;

                        // Triangle
                        ImVec2 p1(center.x - spacing - arrowW * 0.5f,
                                  center.y - arrowH);
                        ImVec2 p2(center.x - spacing - arrowW * 0.5f,
                                  center.y + arrowH);
                        ImVec2 p3(center.x - spacing + arrowW * 0.5f, center.y);

                        dl->AddTriangleFilled(p1, p2, p3, IM_COL32_WHITE);

                        // Bar (vertical line to the right of triangle)
                        float barX = center.x + spacing + arrowW * 0.5f;
                        dl->AddRectFilled(
                            ImVec2(barX - 1.0f, center.y - arrowH),
                            ImVec2(barX + 1.0f, center.y + arrowH),
                            IM_COL32_WHITE);
                    },
                    IM_COL32(60, 60, 60, 255))) {

                m_off_line_frame_index = max_frame_count - 1;
            }

            ImGui::SameLine(0.0f, 10.0f);

            if (DrawIconButton("Stop", [](ImDrawList *dl, ImVec2 min,
                                          ImVec2 max) {
                    ImVec2 center = (min + max) * 0.5f;
                    float side = (max.x - min.x) * 0.4f;
                    ImVec2 pMin(center.x - side * 0.5f, center.y - side * 0.5f);
                    ImVec2 pMax(center.x + side * 0.5f, center.y + side * 0.5f);
                    dl->AddRectFilled(pMin, pMax, IM_COL32_WHITE);
                })) {
                m_off_line_frame_index = 0;
                m_is_playing = false;
                m_fps_frame_received = 0;
                CameraStop();
            }

            NewLine(5.0f);
            ImGui::SliderInt("Frame #", (int *)&m_off_line_frame_index, 0, max_frame_count - 1,"#: %d");
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
            PointCloudReset();
        }
        NewLine(5.0f);
        ImGui::SliderInt("Point Size", &m_point_size, 1, 10, "Point Size: %d px");

        NewLine(5.0f);

        static uint32_t selected;

        ImGui::RadioButton("Depth Colour", selected == 0);
        if (ImGui::IsItemClicked())
            selected = 0;
        ImGui::RadioButton("AB Colour", selected == 1);
        if (ImGui::IsItemClicked())
            selected = 1;

        m_view_instance->setPointCloudColour(selected);

        NewLine(5.0f);

        DrawBarLabel("Active Brightness");
        NewLine(5.0f);
        bool logImage = m_view_instance->getLogImage();
        bool autoScale = m_view_instance->getAutoScale();

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

        m_view_instance->setLogImage(logImage);
        m_view_instance->setAutoScale(autoScale);
        NewLine(5.0f);

        DrawBarLabel("Configuration Parameters");
        NewLine(5.0f);
        //ImGui::Toggle("Show Ini Window", &show_ini_window);
        show_ini_window = true;
        if (show_ini_window && !m_off_line) {
            ShowIniWindow(&show_ini_window);
        }
    }

    ImGui::End();
}

void ADIMainWindow::ShowIniWindow(bool* p_open) {
    aditof::Status status;

    static float abThreshMin = 0;
    static float abSumThresh = 0;
    static float confThresh = 0;
    static float radialThreshMin = 0;
    static float radialThreshMax = 0;
    static bool jblfApplyFlag = false;
    static int jblfWindowSize = 0;
    static float jblfGaussianSigma = 0;
    static float jblfExponentialTerm = 0;
    static float jblfMaxEdge = 0;
    static float jblfABThreshold = 0;
    static int headerSize = 0;
    static bool metadata = false;

    if (m_is_playing && m_ini_params.empty()) {
        status = GetActiveCamera()->getFrameProcessParams(m_ini_params);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Could not get ini params";
        }
        else {
            abThreshMin = std::stof(m_ini_params["abThreshMin"]);
            abSumThresh = std::stof(m_ini_params["abThreshMin"]);
            confThresh = std::stof(m_ini_params["confThresh"]);
            radialThreshMin = std::stof(m_ini_params["radialThreshMin"]);
            radialThreshMax = std::stof(m_ini_params["radialThreshMax"]);
            if (static_cast<int>(
                std::round(std::stof(m_ini_params["jblfApplyFlag"]))) == 1) {
                jblfApplyFlag = true;
            }
            else {
                jblfApplyFlag = false;
            }

            jblfWindowSize = static_cast<int>(
                std::round(std::stof(m_ini_params["jblfWindowSize"])));
            jblfGaussianSigma = std::stof(m_ini_params["jblfGaussianSigma"]);
            jblfExponentialTerm = std::stof(m_ini_params["jblfExponentialTerm"]);
            jblfMaxEdge = std::stof(m_ini_params["jblfMaxEdge"]);
            jblfABThreshold = std::stof(m_ini_params["jblfABThreshold"]);
            headerSize = static_cast<int>(
                std::round(std::stof(m_ini_params["headerSize"])));
            if (headerSize == 128) {
                metadata = true;
            }
            else {
                metadata = false;
            }
        }
    }

    ImGui::PushItemWidth(140 * m_dpi_scale_factor);
    ImGui::InputFloat("abThreshMin", &abThreshMin);
    if (abThreshMin < 0 || abThreshMin > 65535) {
        if (m_last_ini_params["abThreshMin"] != std::to_string(abThreshMin)) {
            LOG(ERROR)
                << "Invalid abThreshMin value. Valid values [0 - 65535]";
            m_last_ini_params["abThreshMin"] = std::to_string(abThreshMin);
        }
        IniParamWarn("abThreshMin", "Valid value: [0 - 65535]");
    }
    ImGui::InputFloat("abSumThresh", &abSumThresh);
    ImGui::InputFloat("confThresh", &confThresh);
    if (confThresh < 0 || confThresh > 255) {
        if (m_last_ini_params["confThresh"] != std::to_string(confThresh)) {
            LOG(ERROR)
                << "Invalid confThresh value. Valid values [0 - 255]";
            m_last_ini_params["confThresh"] = std::to_string(confThresh);
        }

        IniParamWarn("confThresh", "Valid value: [0 - 255]");
    }
    ImGui::InputFloat("radialThreshMin", &radialThreshMin);
    if (radialThreshMin < 0 || radialThreshMin > 65535) {
        if (m_last_ini_params["radialThreshMin"] !=
            std::to_string(radialThreshMin)) {
            LOG(ERROR) << "Invalid radialThreshMin value. Valid values [0 "
                "- 65535]";
            m_last_ini_params["radialThreshMin"] =
                std::to_string(radialThreshMin);
        }
        IniParamWarn("radialThreshMin", "Valid value:[0 - 65535]");
    }
    if (radialThreshMin >= radialThreshMax) {
        if (m_last_ini_params["radialThreshMin"] !=
            std::to_string(radialThreshMin)) {
            LOG(ERROR)
                << "radialThreshMin should be less than radialThreshMax";
            m_last_ini_params["radialThreshMin"] =
                std::to_string(radialThreshMin);
        }
        IniParamWarn(
            "radialThreshMin",
            "radialThreshMin should be\nless than radialThreshMax");
    }
    ImGui::InputFloat("radialThreshMax", &radialThreshMax);
    if (radialThreshMax < 0 || radialThreshMax > 65535) {
        if (m_last_ini_params["radialThreshMax"] !=
            std::to_string(radialThreshMax)) {
            LOG(ERROR) << "Invalid radialThreshMax value. Valid values [0 "
                "- 65535]";
            m_last_ini_params["radialThreshMax"] =
                std::to_string(radialThreshMax);
        }
        IniParamWarn("radialThreshMax", "Valid values [0 - 65535]");
    }
    if (radialThreshMin >= radialThreshMax) {
        if (m_last_ini_params["radialThreshMax"] !=
            std::to_string(radialThreshMax)) {
            LOG(ERROR)
                << "radialThreshMax should be greater than radialThreshMin";
            m_last_ini_params["radialThreshMax"] =
                std::to_string(radialThreshMax);
        }
        IniParamWarn(
            "radialThreshMax",
            "radialThreshMax should be\ngreater than radialThreshMin");
    }
    ImGui::Checkbox("jblfApplyFlag", &jblfApplyFlag);
    ImGui::InputInt("jblfWindowSize", &jblfWindowSize);
    if (jblfWindowSize != 3 && jblfWindowSize != 5 && jblfWindowSize != 7) {
        if (m_last_ini_params["jblfWindowSize"] !=
            std::to_string(jblfWindowSize)) {
            LOG(ERROR)
                << "Invalid jblfWindowSize value. Valid values [3, 5, 7]";
            m_last_ini_params["jblfWindowSize"] =
                std::to_string(jblfWindowSize);
        }

        IniParamWarn("jblfWindowSize", "Valid value: [3, 5, 7]");
    }
    ImGui::InputFloat("jblfGaussianSigma", &jblfGaussianSigma);
    if (jblfGaussianSigma < 0 || jblfGaussianSigma > 65535) {
        if (m_last_ini_params["jblfGaussianSigma"] !=
            std::to_string(jblfGaussianSigma)) {
            LOG(ERROR) << "Invalid jblfGaussianSigma value. Valid values "
                "[0 - 65535]";
            m_last_ini_params["jblfGaussianSigma"] =
                std::to_string(jblfGaussianSigma);
        }
        IniParamWarn("jblfGaussianSigma", "Valid value: [0 - 65535]");
    }
    ImGui::InputFloat("jblfExponentialTerm", &jblfExponentialTerm);
    if (jblfExponentialTerm < 0 || jblfExponentialTerm > 255) {
        if (m_last_ini_params["jblfExponentialTerm"] !=
            std::to_string(jblfExponentialTerm)) {
            LOG(ERROR)
                << "Invalid jblfExponentialTerm value. Valid values [0 "
                "- 255]";
            m_last_ini_params["jblfExponentialTerm"] =
                std::to_string(jblfExponentialTerm);
        }
        IniParamWarn("jblfExponentialTerm", "Valid value: [0 - 255]");
    }
    ImGui::InputFloat("jblfMaxEdge", &jblfMaxEdge);
    if (jblfMaxEdge < 0 || jblfMaxEdge > 63) {
        if (m_last_ini_params["jblfMaxEdge"] != std::to_string(jblfMaxEdge)) {
            LOG(ERROR) << "Invalid jblfMaxEdge value. Valid values [0 "
                "- 63]";
            m_last_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
        }
        IniParamWarn("jblfMaxEdge", "Valid value: [0 - 63]");
    }
    ImGui::InputFloat("jblfABThreshold", &jblfABThreshold);
    if (jblfABThreshold < 0 || jblfABThreshold > 131071) {
        if (m_last_ini_params["jblfABThreshold"] !=
            std::to_string(jblfABThreshold)) {
            LOG(ERROR) << "Invalid jblfABThreshold value. Valid values [0 "
                "- 131071]";
            m_last_ini_params["jblfABThreshold"] =
                std::to_string(jblfABThreshold);
        }
        IniParamWarn("jblfABThreshold", "Valid value: [0 - 131071]");
    }
    ImGui::Checkbox("Metadata Over AB", &metadata);

    // modify ini params
    m_modified_ini_params["abThreshMin"] = std::to_string(abThreshMin);
    m_modified_ini_params["abSumThresh"] = std::to_string(abSumThresh);
    m_modified_ini_params["confThresh"] = std::to_string(confThresh);
    m_modified_ini_params["radialThreshMin"] =
        std::to_string(radialThreshMin);
    m_modified_ini_params["radialThreshMax"] =
        std::to_string(radialThreshMax);
    if (jblfApplyFlag) {
        m_modified_ini_params["jblfApplyFlag"] = std::to_string(1);
    }
    else {
        m_modified_ini_params["jblfApplyFlag"] = std::to_string(0);
    }
    m_modified_ini_params["jblfWindowSize"] = std::to_string(jblfWindowSize);
    m_modified_ini_params["jblfGaussianSigma"] =
        std::to_string(jblfGaussianSigma);
    m_modified_ini_params["jblfExponentialTerm"] =
        std::to_string(jblfExponentialTerm);
    m_modified_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
    m_modified_ini_params["jblfABThreshold"] =
        std::to_string(jblfABThreshold);

    // keep ini param input from last time to skip repetitve error log messages
    m_last_ini_params["abThreshMin"] = std::to_string(abThreshMin);
    m_last_ini_params["confThresh"] = std::to_string(confThresh);
    m_last_ini_params["radialThreshMin"] = std::to_string(radialThreshMin);
    m_last_ini_params["radialThreshMax"] = std::to_string(radialThreshMax);
    m_last_ini_params["jblfWindowSize"] = std::to_string(jblfWindowSize);
    m_last_ini_params["jblfGaussianSigma"] =
        std::to_string(jblfGaussianSigma);
    m_last_ini_params["jblfExponentialTerm"] =
        std::to_string(jblfExponentialTerm);
    m_last_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
    m_last_ini_params["jblfABThreshold"] = std::to_string(jblfABThreshold);

    if (ImGui::Button("Modify")) {
        // stop streaming
        m_is_playing = false;
        m_fps_first_frame_number = 0;
        m_fps_frame_received = 0;
        CameraStop();

        m_use_modified_ini_params = true;

        // restart streaming
        m_view_selection_changed = m_view_selection;
        m_is_playing = true;
    }
}
void ADIMainWindow::IniParamWarn(std::string variable, std::string validVal) {
    ImGui::Text("Invalid %s value.", variable.c_str());
    ImGui::Text(validVal.c_str());
}