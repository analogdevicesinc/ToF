/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/
#ifdef _WIN32

#include <vector>
#include <windows.h>

int main(int argc, char **argv);

// Boilerplate required to make the app work as a Windows GUI application (rather than
// as a console application); just hands control off to the normal main function.
//
// Needs to be in a separate file because some stuff in windows.h doesn't play nice with
// some stuff in glfw.h.
//
int CALLBACK
WinMain(_In_ HINSTANCE hInstance, _In_ HINSTANCE hPrevInstance,
        _In_ LPSTR lpCmdLine, // NOLINT(readability-non-const-parameter)
        _In_ int nCmdShow) {
    UNREFERENCED_PARAMETER(hInstance);
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);
    UNREFERENCED_PARAMETER(nCmdShow);

    int argc = __argc;
    std::vector<char *> argv;

    // Add argv[0], which is the path to the executable
    //
    argv.emplace_back(__argv[0]);

    // Windows-specific defaults that are overrideable by command-line
    //

    // Set default DPI
    //
    SetProcessDPIAware();
    constexpr uint16_t NormalDpi = 96;
    uint16_t systemDPI = GetDpiForSystem();
    if (systemDPI > NormalDpi) {
        argc++;
        argv.emplace_back(const_cast<char *>("--HIGHDPI"));
    } else {
        argc++;
        argv.emplace_back(const_cast<char *>("--NORMALDPI"));
    }

    main(argc, &argv[0]);
    fclose(stderr);
}
#endif
