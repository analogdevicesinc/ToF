/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*  Portions Copyright (c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

// Associated header
//
#include "ADIImGUIExtensions.h"

// System headers
//
#include <functional>
#include <sstream>

#include "imgui.h"

namespace {
std::string ConvertToVerticalText(const char *str) {
    std::stringstream ss;
    bool first = true;
    while (*str) {
        if (first) {
            first = false;
        } else {
            ss << "\n";
        }

        ss << *str;
        ++str;
    }
    return ss.str();
}
} // namespace

namespace adiMainWindow {
namespace ImGuiExtensions {

bool ADIButton(const char *label, const bool enabled) {
    return ADIButton(label, ImVec2(0, 0), enabled);
}

bool ADIButton(const char *label, const ImVec2 &size, bool enabled) {
    return ShowDisableableControl<bool>(
        [label, &size]() { return ImGui::Button(label, size); }, enabled);
}

bool ADICheckbox(const char *label, bool *checked, const bool enabled) {
    return ShowDisableableControl<bool>(
        [label, checked]() { return ImGui::Checkbox(label, checked); },
        enabled);
}

bool ADIRadioButton(const char *label, bool active, bool enabled) {
    return ShowDisableableControl<bool>(
        [label, active]() { return ImGui::RadioButton(label, active); },
        enabled);
}

bool ADIRadioButton(const char *label, int *v, int vButton, bool enabled) {
    return ShowDisableableControl<bool>(
        [label, v, vButton]() { return ImGui::RadioButton(label, v, vButton); },
        enabled);
}

bool ADIInputScalar(const char *label, ImGuiDataType dataType, void *dataPtr,
                    const void *step, const void *stepFast, const char *format,
                    bool enabled) {
    return ShowDisableableControl<bool>(
        [&]() {
            return ImGui::InputScalar(label, dataType, dataPtr, step, stepFast,
                                      format);
        },
        enabled);
}

void ADIVText(const char *s) {
    const std::string vLabel = ConvertToVerticalText(s);
    ImGui::Text("%s", vLabel.c_str());
}

void ADIShowTooltip(const char *msg, bool show) {
    if (show && ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(msg);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

} // namespace ImGuiExtensions
} // namespace adiMainWindow