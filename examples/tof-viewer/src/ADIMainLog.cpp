/********************************************************************************/
/*                                                                              */
/*  Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*   Portions Copyright (c) 2020 Analog Devices Inc.							*/
/*  Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#include "ADIMainWindow.h"

using namespace adiMainWindow;

void ADIMainWindow::ShowLogWindow(bool *p_open) {
    SetWindowSize(m_main_window_width / m_dpi_scale_factor, 235.0f);
    SetWindowPosition(0, m_main_window_height / m_dpi_scale_factor - 235.0f);
    ImGuiWindowFlags windowFlags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse;
    m_log.Draw("Camera: Log", p_open, windowFlags);

#ifdef __linux__
    fseek(m_file_input, ftell(m_file_input), SEEK_SET);
#endif

    while (fgets(m_buffer, 512, m_file_input)) {
        if (m_buffer != INIT_LOG_WARNING)
            m_log.AddLog(m_buffer, nullptr);
    }
}