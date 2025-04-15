/********************************************************************************/
/*                                                                              */
/*  Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*   Portions Copyright (c) 2020 Analog Devices Inc.							*/
/*  Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#include "ADIMainWindow.h"

using namespace adiMainWindow;

void ADIMainWindow::showLogWindow(bool *p_open) {
    setWindowSize(mainWindowWidth / dpiScaleFactor, 235.0f);
    setWindowPosition(0, mainWindowHeight / dpiScaleFactor - 235.0f);
    ImGuiWindowFlags windowFlags =
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse;
    my_log.Draw("Camera: Log", p_open, windowFlags);

#ifdef __linux__
    fseek(input, ftell(input), SEEK_SET);
#endif

    while (fgets(buffer, 512, input)) {
        if (buffer != INIT_LOG_WARNING)
            my_log.AddLog(buffer, nullptr);
    }
}