/********************************************************************************/
/*                                                                              */
/*  Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*   Portions Copyright (c) 2020 Analog Devices Inc.							*/
/*  Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#include "ADIMainWindow.h"
#include "ADIImGUIExtensions.h"
#include "ADIOpenFile.h"
#include "aditof/version.h"
#include <aditof/status_definitions.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <iostream>
#include <stdio.h>

#include <aditof/system.h>
#include <cJSON.h>
#define EMBED_HDR_LENGTH 128
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#include "psapi.h"
#include <io.h>
#include <windows.h>
/* TODO : Remove <experimental/filesystem> when updating to C++17 or newer */
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#define PATH_SEPARATOR "\\"
#else
#include "filesystem.hpp"
namespace fs = ghc::filesystem;
#include <limits.h>
#define MAX_PATH PATH_MAX
#define PATH_SEPARATOR "/"
#endif

// About OpenGL function loaders: modern OpenGL doesn't have a standard header
// file and requires individual function pointers to be loaded manually. Helper
// libraries are often used for this purpose! Here we are supporting a few
// common ones: gl3w, glew, glad. You may use another loader/header of your
// choice (glext, glLoadGen, etc.), or chose to manually implement your own.
//#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h> // Initialize with gl3wInit()

// Include glfw3.h after our OpenGL definitions
#include <CompanyIcon.png.h>
#include <CompanyLogo.png.h>
#include <GLFW/glfw3.h>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to
// maximize ease of testing and compatibility with old VS compilers. To link
// with VS2010-era libraries, VS2015+ requires linking with
// legacy_stdio_definitions.lib, which we do using this pragma. Your own project
// should not be affected, as you are likely to link with a newer binary of GLFW
// that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) &&                                 \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

using namespace adiMainWindow;

auto startTime = std::chrono::system_clock::now();
static int numProcessors;
static std::string last_mode = "";
std::map<std::string, float> ini_params;
std::map<std::string, float> modified_ini_params;
bool use_modified_ini_params = false;
uint16_t expectedFPS = 0;
GLFWimage icons[1];
GLFWimage logos[1];
GLuint logo_texture;
uint32_t firstFrame = 0;
uint32_t frameRecvd = 0;
ADIMainWindow::ADIMainWindow() : m_skipNetworkCameras(true) {
#if defined(Debug) && defined(_WIN32)
    static HANDLE self;
    SYSTEM_INFO sysInfo;
    FILETIME ftime, fsys, fuser;

    GetSystemInfo(&sysInfo);
    numProcessors = sysInfo.dwNumberOfProcessors;

    GetSystemTimeAsFileTime(&ftime);
    memcpy(&lastCPU, &ftime, sizeof(FILETIME));

    self = GetCurrentProcess();
    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
    memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));
#else
#endif
    /********************/
    struct stat info;
    char *folderName = "log";
    if (stat(folderName, &info) != 0) { // If no folder named log is found
#ifdef _WIN32
        if (_mkdir("log")) { // Create local folder where all logs will be filed
#elif __linux__
        if (mkdir("log",
                  0777)) { // Create local folder where all logs will be filed
#endif
            LOG(ERROR) << "Could not create folder " << folderName;
        } else {
            LOG(INFO) << "Log folder created with name: " << folderName;
        }
    }
    char wholeLogPath[30];
    char timebuff[20];
    time_t timeNow = time(0);
    struct tm *timeinfo;
    time(&timeNow);
    timeinfo = localtime(&timeNow);
    strftime(timebuff, sizeof(timebuff), "%Y%m%d_%H%M%S", timeinfo);
    // Concatenate the whole path
    strcpy(wholeLogPath, folderName);
#ifdef _WIN32
    strcat(wholeLogPath, "\\");
#elif __linux__
    strcat(wholeLogPath, "/");
#endif
    strcat(wholeLogPath, "log_");
    strcat(wholeLogPath, timebuff);
    strcat(wholeLogPath, ".txt");

    /********************/
    stream = freopen(wholeLogPath, "w", stderr); // Added missing pointer
    setvbuf(stream, 0, _IONBF, 0);               // No Buffering
    input = fopen(wholeLogPath, "r");

    //Parse config file for this application
    //Parse config.json
    std::ifstream ifs(DEFAULT_TOOLS_CONFIG_FILENAME);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    cJSON *config_json = cJSON_Parse(content.c_str());

    if (config_json != NULL) {
        // Get option to look or not for network cameras
        const cJSON *json_skip_network_cameras =
            cJSON_GetObjectItemCaseSensitive(config_json,
                                             "skip_network_cameras");
        if (cJSON_IsString(json_skip_network_cameras) &&
            (json_skip_network_cameras->valuestring != NULL)) {
            std::string value = json_skip_network_cameras->valuestring;
            if (value == "on") {
                m_skipNetworkCameras = true;
            } else if (value == "off") {
                m_skipNetworkCameras = false;
            } else {
                LOG(WARNING) << "Invalid value for 'skip_network_cameras'. "
                                "Accepted values: on, off";
            }
        }

        // Get the IP address of the network camera to which the application should try to connect to
        const cJSON *json_camera_ip =
            cJSON_GetObjectItemCaseSensitive(config_json, "camera_ip");
        if (cJSON_IsString(json_camera_ip) &&
            (json_camera_ip->valuestring != NULL)) {
            m_cameraIp = json_camera_ip->valuestring;
            if (!m_cameraIp.empty()) {
                m_cameraIp = "ip:" + m_cameraIp;
            }
        }

        cJSON_Delete(config_json);
    }
    if (!ifs.fail()) {
        ifs.close();
    }
}

ADIMainWindow::~ADIMainWindow() {

    if (isPlaying) {
        stopPlayCCD();
    }

    //Recording flags
    if (view != nullptr && !view->m_ctrl->m_recorder->m_finishRecording) {
        view->m_ctrl->m_recorder->stopRecording();
    }
    if (view != nullptr && !view->m_ctrl->m_recorder->_stopPlayback) {
        view->m_ctrl->m_recorder->stopPlayback();
        stopPlayCCD();
    }

    // imGUI disposing
    ImGui::GetIO().IniFilename = NULL;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    if (initCameraWorker.joinable()) {
        initCameraWorker.join();
    }
    //fclose(stderr);
}

double ADIMainWindow::getCurrentValue() {
    double percent = 0;
#if defined(Debug) && defined(_WIN32)
    static ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
    FILETIME ftime, fsys, fuser;
    ULARGE_INTEGER now, sys, user;

    GetSystemTimeAsFileTime(&ftime);
    memcpy(&now, &ftime, sizeof(FILETIME));

    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&sys, &fsys, sizeof(FILETIME));
    memcpy(&user, &fuser, sizeof(FILETIME));
    percent = (sys.QuadPart - lastSysCPU.QuadPart) +
              (user.QuadPart - lastUserCPU.QuadPart);
    percent /= (now.QuadPart - lastCPU.QuadPart);
    percent /= numProcessors;
    lastCPU = now;
    lastUserCPU = user;
    lastSysCPU = sys;

#endif
    return percent * 100;
}

static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

bool ADIMainWindow::startImGUI(const ADIViewerArgs &args) {
    // Setup window
    glfwSetErrorCallback(glfw_error_callback); //Error Management
    if (!glfwInit()) {
        return false;
    }

    // Decide GL+GLSL versions
#if __APPLE__
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
#endif

    std::string version = aditof::getApiVersion();
    std::string _title = "Analog Devices, Inc. Time of Flight Main Window v" +
                         version; //Default name

    // Create window with graphics context
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    window = glfwCreateWindow(1280, 1024, _title.c_str(), NULL, NULL);

    if (window == NULL) {
        return false;
    }

    //Load from Memory
    //Company Icon Logo
    int components;
    size_t logo_size = sizeof(CompanyLogo_png);
    size_t icon_size = sizeof(CompanyIcon_png);
    stbi_uc const *company_logo_buffer = CompanyLogo_png;
    stbi_uc const *company_icon_buffer = CompanyIcon_png;
    icons[0].pixels =
        stbi_load_from_memory(company_icon_buffer, icon_size, &icons[0].width,
                              &icons[0].height, &components, 0);
    glfwSetWindowIcon(window, 1, icons); //Set the found Icon
    stbi_image_free(icons[0].pixels);    //free up the memory

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
    bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
    bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
    bool err = gladLoadGL() == 0;
#else
    bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader
                      // is likely to requires some form of initialization.
#endif
    if (err) {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return false;
    }
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;			// Enable Docking
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    // Keyboard Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; //
    // Enable Gamepad Controls

    if (args.HighDpi) {
        dpiScaleFactor = HIGHDPISCALAR;
        _isHighDPI = true;
    } else {
        dpiScaleFactor = NORMALDPISCALAR;
        _isHighDPI = false;
    }
    setDpi();

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //OR
    // ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    m_controller = std::make_shared<adicontroller::ADIController>(
        std::vector<std::shared_ptr<aditof::Camera>>());
    RefreshDevices();

    //Look for Company Logo
    // Create a OpenGL texture identifier
    logo_texture = -1;
    logos[0].pixels =
        stbi_load_from_memory(company_logo_buffer, logo_size, &logos[0].width,
                              &logos[0].height, &components, 0);
    glGenTextures(1, &logo_texture);
    glBindTexture(GL_TEXTURE_2D, logo_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Upload pixels into texture
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, logos[0].width, logos[0].height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, logos[0].pixels);
    stbi_image_free(logos[0].pixels); //free up the memory

    /**************/

    return true;
}

void ADIMainWindow::setDpi() {
    ImGui::GetStyle().ScaleAllSizes(dpiScaleFactor);

    // ImGui doesn't automatically scale fonts, so we have to do that ourselves
    //
    ImFontConfig fontConfig;
    constexpr float defaultFontSize = 13.0f;
    fontConfig.SizePixels = defaultFontSize * dpiScaleFactor;
    ImGui::GetIO().Fonts->AddFontDefault(&fontConfig);

    glfwGetWindowSize(window, &mainWindowWidth, &mainWindowHeight);
    mainWindowWidth = static_cast<int>(mainWindowWidth * dpiScaleFactor);
    mainWindowHeight = static_cast<int>(mainWindowHeight * dpiScaleFactor);
    glfwSetWindowSize(window, mainWindowWidth, mainWindowHeight);
}

static double cpuUsage;

void ADIMainWindow::render() {
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f); //Main Window Color
    // Main imGUI loop
    while (!glfwWindowShouldClose(window)) {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
        // tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data
        // to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
        // data to your main application. Generally you may always pass all
        // inputs to dear imgui, and hide them from your application based on
        // those two flags.
        glfwGetWindowSize(window, &mainWindowWidth, &mainWindowHeight);
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        handleInterruptCallback();
        /***************************************************/
        //Create windows here:
        showMainMenu();
        showLogoWindow();
        if (isPlaying) {
            PlayCCD(modeSelection, viewSelection);
            computeFPS(fps);
            if (view->m_ctrl->panicStop) {
                stopPlayCCD();

                aditof::Status status = aditof::Status::OK;
                auto camera =
                    getActiveCamera(); //already initialized on constructor

                int chipStatus, imagerStatus;
                status = camera->adsd3500GetStatus(chipStatus, imagerStatus);
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Failed to read chip status!";
                } else {
                    LOG(WARNING) << "Chip status error code: " << chipStatus;
                    LOG(WARNING)
                        << "Imager status error code: " << imagerStatus;
                }
            }

        } else if (isPlayRecorded) {
            PlayRecorded();
        }

        /***************************************************/
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        /*glClearColor(clear_color.x, clear_color.y, clear_color.z,
					 clear_color.w);*/
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}

void ADIMainWindow::handleInterruptCallback() {
    aditof::SensorInterruptCallback cb = [this](aditof::Adsd3500Status status) {
        LOG(WARNING) << "status: " << status;
        ImGui::Begin("Interrupt");
        ImGui::Text("%i", status);
    };
    aditof::Status ret_status = aditof::Status::OK;
    auto camera = getActiveCamera();
    if (!camera) {
        return;
    }
    ret_status = camera->getSensor()->adsd3500_register_interrupt_callback(cb);
    if (ret_status != aditof::Status::OK) {
        my_log.AddLog("Could not register interrupt callback");
        return;
    }
}

void ADIMainWindow::showMainMenu() {
    static bool show_app_log = true;
    static bool show_ini_window = false;

    if (show_app_log)
        showLogWindow(&show_app_log);
    if (show_ini_window)
        showIniWindow(&show_ini_window);

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Open")) {
            showDeviceMenu();
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(window, true);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools")) {
            showRecordMenu();
            showPlaybackMenu();
            ImGui::Separator();
            ImGui::MenuItem("Debug Log", NULL, &show_app_log);
            ImGui::MenuItem("Ini Params", NULL, &show_ini_window);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void ADIMainWindow::showRecordMenu() {
    if (ImGui::BeginMenu("Record Options")) {
        ImGuiExtensions::ButtonColorChanger colorChangerStartRec(
            customColorPlay, isPlaying);
        //Allow the user to choose from 1 to 120 frames. Default value is 5 frames
        ImGui::InputInt("Frames", &recordingSeconds, 1, 300);

        if (recordingSeconds < 1) {
            recordingSeconds = 1;
        } else if (recordingSeconds > 300) {
            recordingSeconds = 300;
        }

        ImGui::NewLine();

        if (ImGuiExtensions::ADIButton("Start Recording",
                                       !isRecording && !isPlayRecorded)) {
            fs::path NPath = fs::current_path();
            std::string tempPath = NPath.string();
            char time_buffer[128];
            struct tm timeinfo;
            time_t rawtime;
            time(&rawtime);
#ifdef _WIN32
            localtime_s(&timeinfo, &rawtime);
#else
            localtime_r(&rawtime, &timeinfo);
#endif
            strftime(time_buffer, sizeof(time_buffer), "%Y%m%d%H%M", &timeinfo);
            tempPath += "\\frames" + std::string(time_buffer);

            int filterIndex = 0;
            char tempbuff[MAX_PATH];
            tempPath.copy(tempbuff, tempPath.length(), 0);
            tempbuff[tempPath.length()] = '\0';
            std::string saveFile = getADIFileName(NULL, tempbuff, filterIndex);
            //Check if filename exists and format is corrct
            if (!saveFile.empty() && filterIndex) {
                if (!isPlaying && filterIndex) {
                    //"Press" the play button, in case it is not pressed.
                    PlayCCD(modeSelection,
                            viewSelection); //Which ever is currently selected
                    isPlaying = true;
                }
                //setting binary save option
                view->m_ctrl->m_saveBinaryFormat = view->getSaveBinaryFormat();
                view->m_ctrl->startRecording(saveFile, view->frameHeight,
                                             view->frameWidth,
                                             recordingSeconds);
                isRecording = true;
            }
        }

        ImGui::SameLine();
        ImGuiExtensions::ButtonColorChanger colorChangerRecStop(customColorStop,
                                                                isRecording);
        if (ImGuiExtensions::ADIButton("Stop Recording", isRecording)) {
            view->m_ctrl->stopRecording();
            isRecording = false;
            isPlayRecorded = false;
            firstFrame = 0;
            stopPlayCCD(); //TODO: Create a Stop ToF camera
            isPlaying = false;
        }

        /* AS 2023-6-6: Temporarily disable.
                ImGui::NewLine();
                ImGui::Checkbox("Save Binary records", &m_saveBinaryFormatTmp);
                if (view != NULL) {
                    view->setSaveBinaryFormat(m_saveBinaryFormatTmp);
                }
                */
        ImGui::EndMenu();
    }
}

void ADIMainWindow::showDeviceMenu() {
    if (ImGui::BeginMenu("Device")) {
        ImGuiExtensions::ADIComboBox("Device", "(No available devices)",
                                     ImGuiComboFlags_None, m_connectedDevices,
                                     &m_selectedDevice, _isOpenDevice);
        //If a device is found, then set the first one found
        if (!m_connectedDevices.empty() && m_selectedDevice == -1) {
            m_selectedDevice = 0;
            _isOpenDevice = true;
        }

        if (!m_connectedDevices.empty()) {
            ImGuiExtensions::ADIComboBox(
                "Config", "No Config Files", ImGuiSelectableFlags_None,
                m_configFiles, &configSelection, _isOpenDevice);
        }

        bool _noConnected = m_connectedDevices.empty();
        if (ImGuiExtensions::ADIButton("Refresh", _noConnected)) {
            _isOpenDevice = false;
            cameraWorkerDone = false;
            RefreshDevices();
        }

        ImGui::SameLine();

        const bool openAvailable = !m_connectedDevices.empty();
        {

            ImGuiExtensions::ButtonColorChanger colorChanger(
                ImGuiExtensions::ButtonColor::Green, openAvailable);
            if (ImGuiExtensions::ADIButton("Open",
                                           /*openAvailable*/ _isOpenDevice &&
                                               m_configFiles.size() > 0) &&
                0 <= m_selectedDevice) {
                if (isPlayRecorded) {
                    stopPlayback();
                }

                _isOpenDevice = false;
                initCameraWorker =
                    std::thread(std::bind(&ADIMainWindow::InitCamera, this));
            }
        }
        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Close")) {
            stopPlayback();
            stopPlayCCD();
            cameraWorkerDone = false;
            m_cameraModes.clear();
            _cameraModes.clear();
            if (initCameraWorker.joinable()) {
                initCameraWorker.join();
            }
            RefreshDevices();
        }
        ImGui::EndMenu();
    }

#if defined(Debug) && defined(_WIN32)
    cpuUsage = getCurrentValue();
    std::string cpu = std::to_string(cpuUsage);
    ImGui::Text("CPU Usage: %s %%", cpu.c_str());

    /*Total Virtual Memory*/
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile;

    /*Total Virtual Memory currently used*/
    DWORDLONG virtualMemUsed =
        memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;

    /*Virtual memory by current process*/
    PROCESS_MEMORY_COUNTERS_EX pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS *)&pmc,
                         sizeof(pmc));
    SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

    /*Physical Memory currently used by current process:*/
    SIZE_T physMemUsedByMe = pmc.WorkingSetSize;

    /*Physical Memory currently used*/
    DWORDLONG physMemUsed = memInfo.ullTotalPhys - memInfo.ullAvailPhys;

    SIZE_T totalMemUsedByProcess = virtualMemUsedByMe - physMemUsedByMe;
    //TODO: Find an accurate way of displaying the memory
    ImGui::Text("Memory Used by Process: %s MB",
                std::to_string((double)physMemUsedByMe / 1000000).c_str());
#endif

    if (cameraWorkerDone) {
        _isOpenDevice = false;
        if (ImGui::BeginMenu("ToF Camera Options")) {
            ImGuiExtensions::ADIComboBox("Mode", "Select Mode",
                                         ImGuiSelectableFlags_None,
                                         m_cameraModes, &modeSelection, true);

            ImGui::NewLine();
            ImGui::Text("View Options:");
            ImGuiExtensions::ADIRadioButton("Active Brightness and Depth",
                                            &viewSelection, 0);
            ImGuiExtensions::ADIRadioButton("Point Cloud and Depth",
                                            &viewSelection, 1);
            ImGui::NewLine();
            ImGui::Text("Video:");

            ImGuiExtensions::ButtonColorChanger colorChangerPlay(
                customColorPlay, !isPlaying);
            if (ImGuiExtensions::ADIButton("Play",
                                           !isPlaying && !isPlayRecorded)) {
                viewSelectionChanged = viewSelection;
                isPlaying = true;
            }
            ImGui::SameLine();
            ImGuiExtensions::ButtonColorChanger colorChangerStop(
                customColorStop, isPlaying);
            if (ImGuiExtensions::ADIButton("Stop", isPlaying)) {
                isPlaying = false;
                isPlayRecorded = false;
                firstFrame = 0;
                frameRecvd = 0;
                stopPlayCCD();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
            ImGui::EndMenu();
        }
        if (view != NULL && view->m_ctrl->recordingFinished() && isRecording) {
            view->m_ctrl->stopRecording();
            isRecording = false;
        }
    }
}

void ADIMainWindow::RefreshDevices() {
    cameraWorkerDone = false;
    m_cameraModes.clear();
    _cameraModes.clear();
    if (initCameraWorker.joinable()) {
        initCameraWorker.join();
    }

    m_selectedDevice = -1;
    m_connectedDevices.clear();
    m_configFiles.clear();
    m_camerasList.clear();

    aditof::Status status;
    status = m_system.getCameraList(m_camerasList);
    for (size_t ix = 0; ix < m_camerasList.size(); ++ix) {
        m_connectedDevices.emplace_back(ix, "ToF Camera " + std::to_string(ix));
    }

    if (!m_skipNetworkCameras) {
        // Add network camera
        m_system.getCameraList(m_camerasList, m_cameraIp);
        if (m_camerasList.size() > 0) {
            int index = m_connectedDevices.size();
            m_connectedDevices.emplace_back(index, "ToF Camera" +
                                                       std::to_string(index));
        }
    }

    if (!m_connectedDevices.empty()) {
        //Search for configuration files with .json extension
        configSelection = -1;
        fs::path _currPath = fs::current_path();
        std::vector<std::string> files;
        getFilesList(_currPath.string(), "*.json", files, false);

        for (size_t fileCnt = 0; fileCnt < files.size(); fileCnt++) {
            m_configFiles.emplace_back(fileCnt, files[fileCnt]);
        }

        if (!m_configFiles.empty() && configSelection == -1) {
            configSelection = 0;
        }
    }
}

void ADIMainWindow::setWindowPosition(float x, float y) {
    x = x * dpiScaleFactor;
    y = y * dpiScaleFactor;
    ImVec2 winPos = {x, y};
    ImGui::SetNextWindowPos(winPos);
}

void ADIMainWindow::setWindowSize(float width, float height) {
    width = width * dpiScaleFactor;
    height = height * dpiScaleFactor;
    ImVec2 m_size = {width, height};
    ImGui::SetNextWindowSize(m_size);
}

void ADIMainWindow::showLogoWindow() {
    //Logo Window
    float info_width = 900.0f;
    float info_height = (_isHighDPI) ? 205.0f : 180.0f;
    setWindowPosition(offsetfromleft + info_width, offsetfromtop);
    setWindowSize(300.0f, info_height);

    ImGui::Begin("Company Logo", NULL,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoScrollbar);

    float scale = 1.0f;
    if (logos[0].pixels != nullptr) {
        if (logos[0].width > 480.0f) {
            if (_isHighDPI) {
                scale = 480.0f / logos[0].width;
            } else {
                scale = (480.0f / logos[0].width) / 4;
            }
        }
        if ((logos[0].height * scale) > 145.0) {
            if (_isHighDPI) {
                scale = 145.0f / logos[0].height;
            } else {
                scale = (145.0f / logos[0].height) / 4;
            }
        }

        ImGui::Image((void *)(intptr_t)logo_texture,
                     ImVec2(logos[0].width * scale, logos[0].height * scale));
    }

    ImGui::End(); //Logo END
}

void ADIMainWindow::showPlaybackMenu() {
    /**********/
    //Playback
    if (ImGui::BeginMenu("Playback Options")) {
        float customColorOpenRec = 0.22f;
        ImGuiExtensions::ButtonColorChanger colorChangerPlay(
            customColorOpenRec, !isPlayRecorded && !isPlaying);

        if (ImGuiExtensions::ADIButton("Open Recording",
                                       !isPlayRecorded && !isPlaying)) {
            int filterIndex = 0;
            std::string path =
                openADIFileName(customFilter.c_str(), NULL, filterIndex)
                    .c_str();
            if (!path.empty() && filterIndex) {

                if (view == NULL) {
                    view = std::make_shared<adiviewer::ADIView>(
                        m_controller, "Record Viewer");
                }

                view->m_ctrl->startPlayback(path, recordingSeconds);
                if (viewSelection == 0) {
                    initOpenGLABTexture();
                } else {
                    initOpenGLPointCloudTexture();
                }
                initOpenGLDepthTexture();

                isPlayRecorded = true;
                _isOpenDevice = false;
                cameraOptionsTreeEnabled = false;

            } else {
                _isOpenDevice = true;
                cameraOptionsTreeEnabled = true;
                LOG(ERROR) << "Non-existing file or unsupported file format "
                              "chosen for playback";
            }
        }

        std::string playbackButtonText =
            isPlayRecordPaused ? "Paused" : "Pause";
        float recordPlaybackColor = customColorPause;
        bool isPlayRecordDone = false;
        if (isPlayRecorded && view && view->m_ctrl->playbackFinished()) {
            isPlayRecordDone = true;
            isPlayRecordPaused = true;
            view->m_ctrl->pausePlayback(true);
            playbackButtonText = "Play";
            recordPlaybackColor = customColorPlay;
        }
        ImGui::SameLine();
        ImGuiExtensions::ButtonColorChanger colorChangerPausePB(
            recordPlaybackColor, isPlayRecorded && isPlayRecordPaused);

        if (ImGuiExtensions::ADIButton(playbackButtonText.c_str(),
                                       isPlayRecorded)) {
            isPlayRecordPaused = !isPlayRecordPaused;
            if (isPlayRecordPaused) {
                view->m_ctrl->pausePlayback(true);
                LOG(INFO) << "Stream has been paused...";
            } else {
                if (isPlayRecordDone) {
                    view->m_ctrl->m_recorder->currentPBPos = 0;
                    isPlayRecordDone = false;
                }
                view->m_ctrl->pausePlayback(false);
                LOG(INFO) << "Streaming.";
            }
            view->m_ctrl->playbackPaused();
        }
        ImGui::SameLine();
        if (view != nullptr && view->m_ctrl->m_recorder->_stopPlayback) {
            stopPlayback();
            view->m_ctrl->m_recorder->_stopPlayback = false;
        }

        ImGuiExtensions::ButtonColorChanger colorChangerStopPB(customColorStop,
                                                               isPlayRecorded);
        if (ImGuiExtensions::ADIButton("Close Recording", isPlayRecorded)) {
            firstFrame = 0;
            stopPlayback();
        }

        if (isPlayRecorded) {
            ImGui::NewLine();
            rawSeeker =
                (view->m_ctrl->m_recorder->currentPBPos) /
                (((int)view->m_ctrl->m_recorder->m_frameDetails.height) *
                     ((int)view->m_ctrl->m_recorder->m_frameDetails.width) *
                     sizeof(uint16_t) * 6 +
                 EMBED_HDR_LENGTH);
            // LOG(INFO) << "rawSeeker :" << rawSeeker;
            ImGuiExtensions::ADISliderInt(
                "Progress", &rawSeeker, 0,
                (view->m_ctrl->m_recorder->m_numberOfFrames) - 1, "%d", true);
            view->m_ctrl->m_recorder->currentPBPos =
                rawSeeker *
                    (((int)view->m_ctrl->m_recorder->m_frameDetails.height) *
                         ((int)view->m_ctrl->m_recorder->m_frameDetails.width) *
                         sizeof(uint16_t) * 6 +
                     EMBED_HDR_LENGTH) +
                view->m_ctrl->m_recorder->m_sizeOfHeader;
        }
        ImGui::NewLine();
        ImGui::Text("View Options:");
        ImGuiExtensions::ADIRadioButton(
            "Active Brightness and Depth", &viewSelection, 0,
            (displayAB || displayDepth) && !isPlayRecorded);
        ImGuiExtensions::ADIRadioButton("Point Cloud and Depth", &viewSelection,
                                        1, pointCloudEnable && !isPlayRecorded);

        //Playback
        ImGui::EndMenu();
    }
}

void ADIMainWindow::stopPlayback() {
    isPlaying = false;
    isPlayRecorded = false;
    displayDepth = true;
    displayAB = true;
    cameraOptionsTreeEnabled = true;
    _isOpenDevice = true;
    if (view && view->m_ctrl) {
        view->m_ctrl->stopPlayback();
        view->m_ctrl->pausePlayback(false);
    }
    isPlayRecordPaused = false;
    stopPlayCCD();
    //view.reset();
    LOG(INFO) << "Stream has been stopped.";
}

std::string ADIMainWindow::getCurrentPath() {
    fs::path currPath = fs::current_path();
    return currPath.string();
}

void ADIMainWindow::PlayRecorded() {
    ImGuiWindowFlags
        overlayFlags = /*ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |*/
        ImGuiWindowFlags_NoResize | /*ImGuiWindowFlags_AlwaysAutoResize |*/
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    const bool imageIsHovered = ImGui::IsItemHovered();

    displayDepth = true;
    pointCloudEnable = true;
    if (viewSelection == 0) {
        displayAB = true;
        displayPointCloud = false;
        synchronizeDepthABVideo();
    } else {
        displayAB = false;
        displayPointCloud = true;
        synchronizePointCloudVideo();
    }

    if (displayAB) {
        displayActiveBrightnessWindow(overlayFlags);
    }
    if (displayDepth) {
        displayDepthWindow(overlayFlags);
    }
    if (displayPointCloud && viewSelection == 1) {
        displayPointCloudWindow(overlayFlags);
    }
    if (displayTemp) {
        displayInfoWindow(overlayFlags);
    }
}

void ADIMainWindow::stopPlayCCD() {
    captureSeparateEnabled = true;
    captureBlendedEnabled = true;
    setABWinPositionOnce = true;
    setDepthWinPositionOnce = true;
    setPointCloudPositionOnce = true;
    dephtWinCtr = 0;
    if (view) {
        if (view->m_ctrl) {
            view->m_ctrl->StopCapture();
            view->m_ctrl->panicStop = false;
        }
        view->m_capturedFrame = nullptr;
        view->ab_video_data = nullptr;
        view->depth_video_data = nullptr;
        view->pointCloud_video_data = nullptr;
    }
    openGLCleanUp();
    isPlaying = false;
    isPlayRecorded = false;
}

void ADIMainWindow::openGLCleanUp() {
    glDeleteTextures(1, &ab_video_texture);
    glDeleteTextures(1, &depth_video_texture);
    glDeleteTextures(1, &pointCloud_video_texture);
    glDeleteVertexArrays(1, &view->vertexArrayObject);
    glDeleteBuffers(1, &view->vertexBufferObject);
    glDeleteProgram(view->pcShader.Id());
    view->pcShader.RemoveShaders();
}

void ADIMainWindow::showLogWindow(bool *p_open) {
    setWindowSize(mainWindowWidth / dpiScaleFactor, 235.0f);
    setWindowPosition(0, mainWindowHeight / dpiScaleFactor - 235.0f);
    my_log.Draw("Camera: Log", p_open);

    while (fgets(buffer, 512, input)) {
        if (buffer != INIT_LOG_WARNING)
            my_log.AddLog(buffer, nullptr);
    }
}

void ADIMainWindow::showIniWindow(bool *p_open) {
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

    if (isPlaying && ini_params.empty()) {
        status = getActiveCamera()->getIniParams(ini_params);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Could not get ini params";
        } else {
            abThreshMin = ini_params["ab_thresh_min"];
            abSumThresh = ini_params["ab_sum_thresh"];
            confThresh = ini_params["conf_thresh"];
            radialThreshMin = ini_params["radial_thresh_min"];
            radialThreshMax = ini_params["radial_thresh_max"];
            if (static_cast<int>(std::round(ini_params["jblf_apply_flag"])) ==
                1) {
                jblfApplyFlag = true;
            } else {
                jblfApplyFlag = false;
            }

            jblfWindowSize =
                static_cast<int>(std::round(ini_params["jblf_window_size"]));
            jblfGaussianSigma = ini_params["jblf_gaussian_sigma"];
            jblfExponentialTerm = ini_params["jblf_exponential_term"];
            jblfMaxEdge = ini_params["jblf_max_edge"];
            jblfABThreshold = ini_params["jblf_ab_threshold"];
            headerSize = static_cast<int>(std::round(ini_params["headerSize"]));
            if (headerSize == 128) {
                metadata = true;
            } else {
                metadata = false;
            }
        }
    }

    if (!ImGui::Begin("ini Params Window", p_open)) {
        ImGui::End();
        return;
    } else {
        ImGui::PushItemWidth(140 * dpiScaleFactor);
        ImGui::InputFloat("abThreshMin", &abThreshMin);
        if (abThreshMin < 0 || abThreshMin > 65535) {
            LOG(ERROR) << "Invalid abThreshMin value. Valid values [0 - 65535]";
            iniParamWarn("abThreshMin", "Valid value: [0 - 65535]");
        }
        ImGui::InputFloat("abSumThresh", &abSumThresh);
        ImGui::InputFloat("confThresh", &confThresh);
        if (confThresh < 0 || confThresh > 255) {
            LOG(ERROR) << "Invalid confThresh value. Valid values [0 - 255]";
            iniParamWarn("confThresh", "Valid value: [0 - 255]");
        }
        ImGui::InputFloat("radialThreshMin", &radialThreshMin);
        if (radialThreshMin < 0 || radialThreshMin > 65535) {
            LOG(ERROR)
                << "Invalid radialThreshMin value. Valid values [0 - 65535]";
            iniParamWarn("radialThreshMin", "Valid value:[0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            LOG(ERROR) << "radialThreshMin should be less than radialThreshMax";
            iniParamWarn(
                "radialThreshMin",
                "radialThreshMin should be\nless than radialThreshMax");
        }
        ImGui::InputFloat("radialThreshMax", &radialThreshMax);
        if (radialThreshMax < 0 || radialThreshMax > 65535) {
            LOG(ERROR)
                << "Invalid radialThreshMax value. Valid values [0 - 65535]";
            iniParamWarn("radialThreshMax", "Valid values [0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            LOG(ERROR)
                << "radialThreshMax should be greater than radialThreshMin";
            iniParamWarn(
                "radialThreshMin",
                "radialThreshMax should be\ngreater than radialThreshMin");
        }
        ImGui::Checkbox("jblfApplyFlag", &jblfApplyFlag);
        ImGui::InputInt("jblfWindowSize", &jblfWindowSize);
        if (jblfWindowSize != 3 && jblfWindowSize != 5 && jblfWindowSize != 7) {
            LOG(ERROR)
                << "Invalid jblfWindowSize value. Valid values [3, 5, 7]";
            iniParamWarn("jblfWindowSize", "Valid value: [3, 5, 7]");
        }
        ImGui::InputFloat("jblfGaussianSigma", &jblfGaussianSigma);
        if (jblfGaussianSigma < 0 || jblfGaussianSigma > 65535) {
            LOG(ERROR)
                << "Invalid jblfGaussianSigma value. Valid values [0 - 65535]";
            iniParamWarn("jblfGaussianSigma", "Valid value: [0 - 65535]");
        }
        ImGui::InputFloat("jblfExponentialTerm", &jblfExponentialTerm);
        if (jblfExponentialTerm < 0 || jblfExponentialTerm > 255) {
            LOG(ERROR) << "Invalid jblfExponentialTerm value. Valid values [0 "
                          "- 255]";
            iniParamWarn("jblfExponentialTerm", "Valid value: [0 - 255]");
        }
        ImGui::InputFloat("jblfMaxEdge", &jblfMaxEdge);
        if (jblfMaxEdge < 0 || jblfMaxEdge > 63) {
            LOG(ERROR) << "Invalid jblfMaxEdge value. Valid values [0 "
                          "- 63]";
            iniParamWarn("jblfMaxEdge", "Valid value: [0 - 63]");
        }
        ImGui::InputFloat("jblfABThreshold", &jblfABThreshold);
        if (jblfABThreshold < 0 || jblfABThreshold > 131071) {
            LOG(ERROR) << "Invalid jblfABThreshold value. Valid values [0 "
                          "- 131071]";
            iniParamWarn("jblfABThreshold", "Valid value: [0 - 131071]");
        }
        ImGui::Checkbox("Metadata Over AB", &metadata);

        if (ImGui::Button("Modify")) {
            // stop streaming
            {
                isPlaying = false;
                isPlayRecorded = false;
                firstFrame = 0;
                frameRecvd = 0;
                stopPlayCCD();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
            // modify ini params
            modified_ini_params["ab_thresh_min"] = abThreshMin;
            modified_ini_params["ab_sum_thresh"] = abSumThresh;
            modified_ini_params["conf_thresh"] = confThresh;
            modified_ini_params["radial_thresh_min"] = radialThreshMin;
            modified_ini_params["radial_thresh_max"] = radialThreshMax;
            if (jblfApplyFlag) {
                modified_ini_params["jblf_apply_flag"] = 1;
            } else {
                modified_ini_params["jblf_apply_flag"] = 0;
            }
            modified_ini_params["jblf_window_size"] = jblfWindowSize;
            modified_ini_params["jblf_gaussian_sigma"] = jblfGaussianSigma;
            modified_ini_params["jblf_exponential_term"] = jblfExponentialTerm;
            modified_ini_params["jblf_max_edge"] = jblfMaxEdge;
            modified_ini_params["jblf_ab_threshold"] = jblfABThreshold;
            use_modified_ini_params = true;
            // restart streaming
            {
                viewSelectionChanged = viewSelection;
                isPlaying = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            modified_ini_params.clear();
            ini_params.clear();
            use_modified_ini_params = false;
            {
                isPlaying = false;
                isPlayRecorded = false;
                firstFrame = 0;
                frameRecvd = 0;
                stopPlayCCD();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
            {
                viewSelectionChanged = viewSelection;
                isPlaying = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            saveIniFile();
        }
    }
    ImGui::End();
}

void ADIMainWindow::InitCamera() {
    if (view != NULL) { //Reset current imager
        LOG(INFO) << "Imager is reseting.";
        view.reset();
        LOG(INFO) << "Reset successful.";
    }

    std::string version = aditof::getApiVersion();
    //my_log.AddLog("Preparing camera. Please wait...\n");
    LOG(INFO) << "Preparing camera. Please wait...\n";
    m_controller =
        std::make_shared<adicontroller::ADIController>(m_camerasList);

    view = std::make_shared<adiviewer::ADIView>(m_controller,
                                                "ToFViewer " + version);

    aditof::Status status = aditof::Status::OK;
    auto camera = getActiveCamera(); //already initialized on constructor

    if (!camera) {
        LOG(ERROR) << "No cameras found!";
        return;
    }

    status = camera->initialize(m_configFiles[configSelection].second);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

    //Parse config.json
    std::ifstream ifs(m_configFiles[configSelection].second);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    std::vector<std::pair<std::string, int32_t>> device_settings;
    cJSON *config_json = cJSON_Parse(content.c_str());

    if (config_json != NULL) {
        // Get GUI MAX_RANGE file location
        const cJSON *json_min_max_range =
            cJSON_GetObjectItemCaseSensitive(config_json, "MAX_RANGE");
        if (cJSON_IsString(json_min_max_range) &&
            (json_min_max_range->valuestring != NULL)) {
            // Set Max range
            view->maxRange = std::stoi(json_min_max_range->valuestring);
        }
        //Get GUI MIN_RANGE file location
        json_min_max_range =
            cJSON_GetObjectItemCaseSensitive(config_json, "MIN_RANGE");
        if (cJSON_IsString(json_min_max_range) &&
            (json_min_max_range->valuestring != NULL)) {
            // Set Min range
            view->minRange = std::stoi(json_min_max_range->valuestring);
        }
        //Get GUI AB_MAX_RANGE and AB_MIN_RANGE file location
        json_min_max_range =
            cJSON_GetObjectItemCaseSensitive(config_json, "AB_MAX_RANGE");
        if (cJSON_IsString(json_min_max_range) &&
            (json_min_max_range->valuestring != NULL)) {
            uint32_t value =
                (uint32_t)std::stoi(json_min_max_range->valuestring);
            view->setABMaxRange(value);
            view->setUserABMaxState(true);
        } else {
            view->setUserABMaxState(false);
        }
        json_min_max_range =
            cJSON_GetObjectItemCaseSensitive(config_json, "AB_MIN_RANGE");
        if (cJSON_IsString(json_min_max_range) &&
            (json_min_max_range->valuestring != NULL)) {
            uint32_t value =
                (uint32_t)std::stoi(json_min_max_range->valuestring);
            view->setABMinRange(value);
            view->setUserABMinState(true);
        } else {
            view->setUserABMinState(false);
        }
        //Get available modes
        json_min_max_range =
            cJSON_GetObjectItemCaseSensitive(config_json, "modes");
        if (cJSON_IsString(json_min_max_range) &&
            (json_min_max_range->valuestring != NULL)) {
            // Add to _cameraModes the available modes
            std::string cameraElements =
                static_cast<std::string>(json_min_max_range->valuestring);
            std::string delimiter = ",";
            size_t position = 0;
            std::string token = "";
            cameraElements.erase(
                std::remove(cameraElements.begin(), cameraElements.end(), ' '),
                cameraElements
                    .end()); //Cleanup the string to eliminate blank spaces.
            while ((position = cameraElements.find(delimiter)) !=
                   std::string::npos) {
                _cameraModes.emplace_back(cameraElements.substr(0, position));
                std::cout << token << std::endl;
                cameraElements.erase(0, position + delimiter.length());
            }
            //Last element should go here:
            _cameraModes.emplace_back(cameraElements.substr(0, position));
            _usesExternalModeDefinition = true;

        } else {
            _usesExternalModeDefinition = false;
        }

        cJSON_Delete(config_json);
    }
    if (!ifs.fail()) {
        ifs.close();
    }

    if (!_usesExternalModeDefinition)
        camera->getAvailableFrameTypes(_cameraModes);

    int modeIndex = 0;
    for (int i = 0; i < _cameraModes.size(); ++i) {
#ifndef ENBABLE_PASSIVE_IR
        if ("pcm" == _cameraModes.at(i)) {
            continue;
        }
#endif
        modeSelection = modeIndex;
        m_cameraModes.emplace_back(modeIndex++, _cameraModes.at(i));
    }

    cameraWorkerDone = true;
}

void ADIMainWindow::prepareCamera(std::string mode) {
    aditof::Status status = aditof::Status::OK;
    std::vector<aditof::FrameDetails> frameTypes;

    if (mode.empty()) {
        my_log.AddLog("Error: Invalid camera mode!\n");
        return;
    }

    status = getActiveCamera()->setFrameType(mode);
    if (status != aditof::Status::OK) {
        my_log.AddLog("Could not set camera mode!");
        return;
    }

    if (mode == last_mode) {
        if (!modified_ini_params.empty()) {
            if (use_modified_ini_params) {
                status = getActiveCamera()->setIniParams(modified_ini_params);
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Could not set ini params";
                } else {
                    LOG(INFO) << "Using user defined ini parameters.";
                    use_modified_ini_params = false;
                }
            }
        }
    } else {
        last_mode = mode;
    }

    aditof::CameraDetails camDetails;
    status = getActiveCamera()->getDetails(camDetails);
    int totalCaptures = camDetails.frameType.totalCaptures;

    view->m_ctrl->m_recorder->m_frameDetails.totalCaptures = totalCaptures;

    if (!view->getUserABMaxState()) {
        std::string value;
        getActiveCamera()->getSensor()->getControl("abBits", value);
        view->setABMaxRange(value);
    }

    // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
    status = getActiveCamera()->start();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return;
    }

    my_log.AddLog("Camera ready.\n");
    cameraWorkerDone = true;
}

void ADIMainWindow::PlayCCD(int modeSelect, int viewSelect) {
    ImGuiWindowFlags
        overlayFlags = /*ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |*/
        ImGuiWindowFlags_NoResize | /*ImGuiWindowFlags_AlwaysAutoResize |*/
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    const bool imageIsHovered = ImGui::IsItemHovered();

    if (view->m_ctrl->hasCamera()) {
        // Mode switch or starup
        if (modeSelectChanged != modeSelect || captureSeparateEnabled ||
            !isPlaying) {
            if (modeSelectChanged != modeSelect) {
                view->m_ctrl->StopCapture();
            }

            prepareCamera(m_cameraModes[modeSelection].second);
            openGLCleanUp();
            initOpenGLABTexture();
            initOpenGLDepthTexture();
            initOpenGLPointCloudTexture();

            view->m_ctrl->StartCapture();
            view->m_ctrl->requestFrame();
            captureSeparateEnabled = false;
            modeSelectChanged = modeSelect;

        } else if (viewSelectionChanged != viewSelect) {
            viewSelectionChanged = viewSelect;
            openGLCleanUp();
            initOpenGLABTexture();
            initOpenGLDepthTexture();
            initOpenGLPointCloudTexture();
        }
    }

    switch (viewSelect) {
    case 0: //Show Depth and AB in separate windows
        displayAB = true;
        displayDepth = checkCameraSetToReceiveContent("depth");
        synchronizeDepthABVideo();
        displayActiveBrightnessWindow(overlayFlags);
        if (displayDepth) {
            displayDepthWindow(overlayFlags);
        }
        displayInfoWindow(overlayFlags);
        break;
    case 1: //Point Cloud Window
        displayDepth = checkCameraSetToReceiveContent("depth");
        if (displayDepth) {
            displayAB = false;
            synchronizePointCloudVideo();
            displayPointCloudWindow(overlayFlags);
            displayDepthWindow(overlayFlags);
        } else {
            displayAB = true;
            synchronizeDepthABVideo();
            displayActiveBrightnessWindow(overlayFlags);
        }
        displayInfoWindow(overlayFlags);
        break;
    default:
        break;
    }
}

void ADIMainWindow::displayInfoWindow(ImGuiWindowFlags overlayFlags) {
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
        char formattedIP[20];
        for (int i = 0; i < m_cameraIp.length(); i++)
            formattedIP[i] = toupper(m_cameraIp[i]);
        ImGui::Text(" Camera %s", formattedIP);
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
        auto camera = getActiveCamera();
        if (!camera) {
            LOG(ERROR) << "No camera found";
            return;
        }
        auto frame = view->m_capturedFrame;
        frameRecvd++;
        if (!frame) {
            LOG(ERROR) << "No frame received";
            return;
        }
        uint16_t *frameHeader = nullptr;
        aditof::CameraDetails cameraDetails;
        camera->getDetails(cameraDetails);
        std::string camera_mode = cameraDetails.mode;
        if (camera_mode != "pcm-native") {
            frame->getData("metadata", &frameHeader);
        }
        if (frameHeader && (isPlaying || isRecording)) {
            void *metadata = nullptr;
            metadata = frameHeader + 6; // to get frame number
            int32_t *frameNum = reinterpret_cast<int32_t *>(metadata);
            if (!firstFrame) {
                firstFrame = *frameNum;
            }
            metadata = frameHeader + 12; // to get sensor temperature;
            int32_t *sensorTemp = reinterpret_cast<int32_t *>(metadata);
            metadata = frameHeader + 14; // to get lasor temperature;
            int32_t *laserTemp = reinterpret_cast<int32_t *>(metadata);
            ImGui::Text(" Current FPS: %i", fps);
            if (expectedFPS == 0) {
                aditof::Status status =
                    camera->adsd3500GetFrameRate(expectedFPS);
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Failed to get frame rate.";
                }
            }
            if (expectedFPS) {
                ImGui::SameLine();
                ImGui::Text(" | Expected FPS: %i", expectedFPS);
            }
            uint32_t totalFrames = *frameNum - firstFrame;
            uint32_t frameLost = totalFrames - frameRecvd;
            ImGui::Text(" Number of frames lost: %i", frameLost);
            ImGui::SameLine();
            ImGui::Text(" | Number of frames received: %i", frameRecvd);
            ImGui::Text(" Laser Temperature: %iC", *laserTemp);
            ImGui::SameLine();
            ImGui::Text(" | Sensor Temperature: %iC", *sensorTemp);
        }
        ImGui::NewLine();

        // "Stop" button
        ImGuiExtensions::ButtonColorChanger colorChangerStop(customColorStop,
                                                             isPlaying);
        if (isPlaying && !isRecording) {
            ImGui::Text(" View Options:");
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Active Brightness and Depth",
                                            &viewSelection, 0);
            ImGui::SameLine();
            ImGuiExtensions::ADIRadioButton("Point Cloud and Depth",
                                            &viewSelection, 1);
            if (ImGuiExtensions::ADIButton("Stop Streaming", isPlaying)) {
                isPlaying = false;
                isPlayRecorded = false;
                firstFrame = 0;
                frameRecvd = 0;
                stopPlayCCD();
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
                firstFrame = 0;
                frameRecvd = 0;
                stopPlayCCD();
                if (isRecording) {
                    view->m_ctrl->stopRecording();
                    isRecording = false;
                }
            }
        }
        if (isPlayRecorded) {
            std::string playbackButtonText =
                isPlayRecordPaused ? "Paused" : "Pause";
            float recordPlaybackColor = customColorPause;
            bool isPlayRecordDone = false;
            if (isPlayRecorded && view && view->m_ctrl->playbackFinished()) {
                isPlayRecordDone = true;
                isPlayRecordPaused = true;
                view->m_ctrl->pausePlayback(true);
                playbackButtonText = "Play";
                recordPlaybackColor = customColorPlay;
            }
            ImGui::SameLine();
            ImGuiExtensions::ButtonColorChanger colorChangerPausePB(
                recordPlaybackColor, isPlayRecorded && isPlayRecordPaused);

            if (ImGuiExtensions::ADIButton(playbackButtonText.c_str(),
                                           isPlayRecorded)) {
                isPlayRecordPaused = !isPlayRecordPaused;
                if (isPlayRecordPaused) {
                    view->m_ctrl->pausePlayback(true);
                    LOG(INFO) << "Stream has been paused...";
                } else {
                    if (isPlayRecordDone) {
                        view->m_ctrl->m_recorder->currentPBPos = 0;
                        isPlayRecordDone = false;
                    }
                    view->m_ctrl->pausePlayback(false);
                    LOG(INFO) << "Streaming.";
                }
                view->m_ctrl->playbackPaused();
            }
            ImGui::SameLine();
            if (view != nullptr && view->m_ctrl->m_recorder->_stopPlayback) {
                stopPlayback();
                view->m_ctrl->m_recorder->_stopPlayback = false;
            }

            ImGuiExtensions::ButtonColorChanger colorChangerStopPB(
                customColorStop, isPlayRecorded);
            if (ImGuiExtensions::ADIButton("Close Recording", isPlayRecorded)) {
                firstFrame = 0;
                stopPlayback();
            }
            rawSeeker =
                (view->m_ctrl->m_recorder->currentPBPos) /
                (((int)view->m_ctrl->m_recorder->m_frameDetails.height) *
                     ((int)view->m_ctrl->m_recorder->m_frameDetails.width) *
                     sizeof(uint16_t) * 6 +
                 EMBED_HDR_LENGTH);
            ImGuiExtensions::ADISliderInt(
                "Progress", &rawSeeker, 0,
                (view->m_ctrl->m_recorder->m_numberOfFrames) - 1, "%d", true);
            // LOG(INFO) << "Progress bar rawSeeker: " << rawSeeker;
            view->m_ctrl->m_recorder->currentPBPos =
                rawSeeker *
                    (((int)view->m_ctrl->m_recorder->m_frameDetails.height) *
                         ((int)view->m_ctrl->m_recorder->m_frameDetails.width) *
                         sizeof(uint16_t) * 6 +
                     EMBED_HDR_LENGTH) +
                view->m_ctrl->m_recorder->m_sizeOfHeader;
        }
    }
    ImGui::End();
}

bool ADIMainWindow::displayDataWindow(ImVec2 &displayUpdate, ImVec2 &size) {
    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return false;
    }

    size.x = view->frameWidth * imagescale;
    size.y = view->frameHeight * imagescale;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(size.x, size.y);
    }

    displayUpdate = {static_cast<float>(size.x), static_cast<float>(size.y)};

    size.x /= dpiScaleFactor;
    size.y /= dpiScaleFactor;

    return true;
}

void ADIMainWindow::displayActiveBrightnessWindow(
    ImGuiWindowFlags overlayFlags) {

    ImVec2 size;

    if (displayDataWindow(displayABDimensions, size) == false)
        return;

    dictWinPosition["ab"] = std::array<float, 4>(
        {offsetfromleft,
         dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
         size.y});
    setWindowPosition(dictWinPosition["ab"][0], dictWinPosition["ab"][1]);
    setWindowSize(dictWinPosition["ab"][2] + 40, dictWinPosition["ab"][3] + 40);

    if (ImGui::Begin("Active Brightness Window", nullptr, overlayFlags)) {
        CaptureABVideo();
        bool logImage = view->getLogImage();
        ImGui::Checkbox("Log Image", &logImage);
        view->setLogImage(logImage);
        ImGui::SameLine();
        bool autoScale = view->getAutoScale();
        ImGui::Checkbox("Auto-scale", &autoScale);
        view->setAutoScale(autoScale);

        ImVec2 hoveredImagePixel = InvalidHoveredPixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
                           ImGui::GetIO().MousePos, displayABDimensions);
        RenderInfoPane(hoveredImagePixel, view->ab_video_data, view->frameWidth,
                       ImGui::IsWindowHovered(),
                       ADI_Image_Format_t::ADI_IMAGE_FORMAT_AB16, " ");
    }

    ImGui::End();
}

void ADIMainWindow::displayDepthWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    if (displayDataWindow(displayDepthDimensions, size) == false)
        return;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    sourceDepthImageDimensions = {(float)(view->frameWidth),
                                  (float)(view->frameHeight)};

    if (displayAB)
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["ab"][0] + dictWinPosition["ab"][2],
             dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
             size.y});
    else
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["pc"][0] + dictWinPosition["pc"][2],
             dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
             size.y});
    setWindowPosition(dictWinPosition["depth"][0] + 40,
                      dictWinPosition["depth"][1]);
    setWindowSize(dictWinPosition["depth"][2] + 40,
                  dictWinPosition["depth"][3] + 40);

    std::string title =
        "Depth"; //std::format("Depth: {} x {}", static_cast<uint32_t>(view->frameWidth), static_cast<uint32_t>(view->frameWidth));
    if (ImGui::Begin(title.c_str(), nullptr, overlayFlags)) {
        ImVec2 hoveredImagePixel = InvalidHoveredPixel;
        GetHoveredImagePix(hoveredImagePixel, ImGui::GetCursorScreenPos(),
                           ImGui::GetIO().MousePos, displayDepthDimensions);
        RenderInfoPane(hoveredImagePixel, view->depth_video_data,
                       view->frameWidth, ImGui::IsWindowHovered(),
                       ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16, "mm");
    }

    CaptureDepthVideo();
    ImGui::End();
}

void ADIMainWindow::displayPointCloudWindow(ImGuiWindowFlags overlayFlags) {
    ImVec2 size;

    if (displayDataWindow(displayPointCloudDimensions, size) == false)
        return;

    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    sourcePointCloudImageDimensions = {(float)(view->frameWidth),
                                       (float)(view->frameHeight)};

    dictWinPosition["pc"] = std::array<float, 4>(
        {offsetfromleft,
         dictWinPosition["info"][1] + dictWinPosition["info"][3], size.x,
         size.y});

    setWindowPosition(dictWinPosition["pc"][0], dictWinPosition["pc"][1]);
    setWindowSize(dictWinPosition["pc"][2] + 40, dictWinPosition["pc"][3] + 40);

    if (ImGui::Begin("Point Cloud Window", nullptr, overlayFlags)) {
        CapturePointCloudVideo();
        ImGuiExtensions::ADISliderInt("", &pointSize, 1, 10,
                                      "Point Size: %d px");
        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Reset", true)) {
            pointCloudReset();
        }
    }

    //TODO:
    //Create a color bar for Point Cloud
    /*createColorBar({ (float)(view->frameWidth * 1.15), 150.0f },
		{ 200.0, 750.0 });*/
    ImGui::End();
}

void ADIMainWindow::createColorBar(ImVec2 position, ImVec2 size) {
    //TODO: This is an unfinished job. It will be part of the following release on January 2021
    ImDrawList *depthDrawList = ImGui::GetWindowDrawList();
    //Display a rainbow bar, on the top is the farthest and at the bottom the closest.
    //This map is following the HSV convention.
    ImColor colors[] = {ImColor(255, 0, 0),   //Red, far away
                        ImColor(255, 180, 0), //Orange
                        ImColor(255, 255, 0), //Yellow
                        ImColor(0, 255, 0),   //Green, in between
                        ImColor(0, 180, 128), ImColor(0, 255, 255),
                        ImColor(0, 0, 255)}; //Blue, closest

    static const float HUE_PICKER_WIDTH = 20.0f;
    static const float CROSSHAIR_SIZE = 7.0f;
    for (int i = 0; i < 6; ++i) {
        depthDrawList->AddRectFilledMultiColor(
            ImVec2(position.x + size.x + 10, position.y + i * (size.y / 6)),
            ImVec2(position.x + size.x + 10 + HUE_PICKER_WIDTH,
                   position.y + (i + 1) * (size.y / 6)),
            colors[i], colors[i], colors[i + 1], colors[i + 1]);
    }
    //
    ImVec2 p(position.x + size.x, position.y + size.y);
    depthDrawList->AddLine(ImVec2(p.x - CROSSHAIR_SIZE, p.y),
                           ImVec2(p.x - 2, p.y), ImColor(255, 255, 255));
    depthDrawList->AddLine(ImVec2(p.x + CROSSHAIR_SIZE, p.y),
                           ImVec2(p.x + 2, p.y), ImColor(255, 255, 255));
    depthDrawList->AddLine(ImVec2(p.x, p.y + CROSSHAIR_SIZE),
                           ImVec2(p.x, p.y + 2), ImColor(255, 255, 255));
    depthDrawList->AddLine(ImVec2(p.x, p.y - CROSSHAIR_SIZE),
                           ImVec2(p.x, p.y - 2), ImColor(255, 255, 255));
    depthDrawList->AddText(ImGui::GetFont(), 24, position,
                           ImColor(255, 255, 255, 255),
                           "Test Standalone ImGui DrawList");
}

void ADIMainWindow::InitCaptureABDepthVideo() {}

void ADIMainWindow::initOpenGLABTexture() {
    /********************************************/
    //ab Texture
    GLuint ab_texture;
    glGenTextures(1, &ab_texture);
    glBindTexture(GL_TEXTURE_2D, ab_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    ab_video_texture = ab_texture;
    /*************************************************/
}

void ADIMainWindow::initOpenGLDepthTexture() {
    /********************************************/
    //Depth Texture
    GLuint depth_texture;
    glGenTextures(1, &depth_texture);
    glBindTexture(GL_TEXTURE_2D, depth_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    depth_video_texture = depth_texture;
    /*************************************************/
}

void ADIMainWindow::preparePointCloudVertices(unsigned int &vbo,
                                              unsigned int &vao) {
    glGenVertexArrays(1, &vao);
    //Initialize Point Cloud Image
    glGenBuffers(1, &vbo);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(vao);
    //Bind Point Cloud Image
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    //Pass on Image buffer here:
    glBufferData(GL_ARRAY_BUFFER, view->vertexArraySize,
                 view->normalized_vertices,
                 GL_DYNAMIC_DRAW); //GL_STATIC_DRAW

    //Image
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
        (void *)0); //Our image buffer got bigger: [X, Y, Z, R, G, B]
    glEnableVertexAttribArray(0); //Enable [X, Y, Z]

    //Point Cloud Color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1); //Enable [R, G, B]

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
}

void ADIMainWindow::initOpenGLPointCloudTexture() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE); //Enable point size feature

    constexpr char const pointCloudVertexShader[] =
        R"(
				#version 330 core
				layout (location = 0) in vec3 aPos;
				layout (location = 1) in vec3 hsvColor;//Contains R, G, B, values based on HSV standard

				uniform mat4 model;
				uniform mat4 view;
				uniform mat4 projection;

				out vec4 color_based_on_position;

				void main()
				{
					gl_Position = projection * view * model * vec4(aPos, 1.0);
					color_based_on_position = vec4(hsvColor, 1.0);
				}
				)";

    constexpr char const pointCloudFragmentShader[] =
        R"(
				#version 330 core
				out vec4 FragColor;
				in vec4 color_based_on_position;
				void main()
				{
					//FragColor = vec4(0.1f, 0.8f, 0.1f, 1.0f);
					FragColor = color_based_on_position;
				}
				)";

    //Build and compile our shaders
    adiviewer::ADIShader vertexShader(
        GL_VERTEX_SHADER,
        pointCloudVertexShader); //Our vertices (whole image)
    adiviewer::ADIShader fragmentShader(GL_FRAGMENT_SHADER,
                                        pointCloudFragmentShader); //Color map
    view->pcShader.CreateProgram();
    view->pcShader.AttachShader(std::move(vertexShader));
    view->pcShader.AttachShader(std::move(fragmentShader));
    view->pcShader.Link();

    //Get the model, view, and projection index
    view->modelIndex = glGetUniformLocation(view->pcShader.Id(), "model");
    view->viewIndex = glGetUniformLocation(view->pcShader.Id(), "view");
    view->projectionIndex =
        glGetUniformLocation(view->pcShader.Id(), "projection");

    //Initialize Model, View, and Projection Matrices to Identity
    mat4x4_identity(m_view);
    mat4x4_identity(m_projection);
    mat4x4_identity(m_model);

    //Create Frame Buffers to be able to display on the Point Cloud Window.
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // create a color attachment texture
    glGenTextures(1, &pointCloud_video_texture);
    glBindTexture(GL_TEXTURE_2D, pointCloud_video_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mainWindowWidth, mainWindowHeight, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           pointCloud_video_texture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ADIMainWindow::synchronizeDepthABVideo() {

    view->m_capturedFrame = view->m_ctrl->getFrame();

    aditof::FrameDetails frameDetails;
    view->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(view->m_frameCapturedMutex);
    if (displayAB) {
        view->m_abFrameAvailable = true;
    } else {
        view->m_abFrameAvailable = false;
        view->ab_video_data_8bit = nullptr;
    }
    if (displayDepth) {
        view->m_depthFrameAvailable = true;
    } else {
        view->m_depthFrameAvailable = false;
        view->depth_video_data_8bit = nullptr;
    }
    if (displayAB && displayDepth) {
        view->numOfThreads = 2;
    } else {
        view->numOfThreads = 1;
    }

    /************************************/
    //CMOS
    view->frameHeight = frameDetails.height;
    view->frameWidth = frameDetails.width;
    /************************************/

    lock.unlock();
    view->m_frameCapturedCv.notify_all();
    view->m_ctrl->requestFrame();

    /*********************************/
    std::unique_lock<std::mutex> imshow_lock(view->m_imshowMutex);
    view->m_barrierCv.wait(imshow_lock, [&]() {
        return view->m_waitKeyBarrier == view->numOfThreads; /*2*/
        ;
    });
    view->m_waitKeyBarrier = 0;
    /*********************************/
}

void ADIMainWindow::synchronizePointCloudVideo() {
    view->m_capturedFrame = view->m_ctrl->getFrame();

    aditof::FrameDetails frameDetails;
    view->m_capturedFrame->getDetails(frameDetails);
    std::unique_lock<std::mutex> lock(view->m_frameCapturedMutex);

    view->m_depthFrameAvailable = true;
    view->m_pointCloudFrameAvailable = true;
    view->frameHeight = frameDetails.height;
    view->frameWidth = frameDetails.width;

    lock.unlock();
    view->m_frameCapturedCv.notify_all();
    view->m_ctrl->requestFrame();

    /*********************************/
    std::unique_lock<std::mutex> imshow_lock(view->m_imshowMutex);
    view->m_barrierCv.wait(imshow_lock, [&]() {
        return view->m_waitKeyBarrier == view->numOfThreads;
    });
    view->m_waitKeyBarrier = 0;
    /*********************************/
}

void ADIMainWindow::GetHoveredImagePix(ImVec2 &hoveredImagePixel,
                                       ImVec2 imageStartPos, ImVec2 mousePos,
                                       ImVec2 displayDimensions) {
    ImVec2 hoveredUIPixel;
    hoveredUIPixel.x = mousePos.x - imageStartPos.x;
    hoveredUIPixel.y = mousePos.y - imageStartPos.y;

    ImVec2 _displayDepthDimensions = displayDepthDimensions;
    ImVec2 _sourceDepthImageDimensions = sourceDepthImageDimensions;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(_sourceDepthImageDimensions.x, _sourceDepthImageDimensions.y);
    }

    if (hoveredUIPixel.x > _displayDepthDimensions.x ||
        hoveredUIPixel.y > _displayDepthDimensions.y) {
        hoveredImagePixel.x = -1;
        hoveredImagePixel.y = -1;
        return;
    }

    hoveredUIPixel.x = (std::min)(hoveredUIPixel.x, _displayDepthDimensions.x);
    hoveredUIPixel.x = (std::max)(hoveredUIPixel.x, 0.0f);

    hoveredUIPixel.y = (std::min)(hoveredUIPixel.y, _displayDepthDimensions.y);
    hoveredUIPixel.y = (std::max)(hoveredUIPixel.y, 0.0f);

    const float uiCoordinateToImageCoordinateRatio =
        _sourceDepthImageDimensions.x / _displayDepthDimensions.x;

    hoveredImagePixel.x =
        std::round(hoveredUIPixel.x * uiCoordinateToImageCoordinateRatio);
    hoveredImagePixel.y =
        std::round(hoveredUIPixel.y * uiCoordinateToImageCoordinateRatio);

    if (rotationangledegrees == 90) {
        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.y =
            sourceDepthImageDimensions.y - hoveredImagePixel.y;
    } else if (rotationangledegrees == 270) {
        std::swap(hoveredImagePixel.x, hoveredImagePixel.y);
        hoveredImagePixel.x =
            sourceDepthImageDimensions.x - hoveredImagePixel.x;
    } else if (rotationangledegrees == 180) {
        hoveredImagePixel.x =
            _sourceDepthImageDimensions.x - hoveredImagePixel.x;
        hoveredImagePixel.y =
            _sourceDepthImageDimensions.y - hoveredImagePixel.y;
    }
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

static inline ImVec2 operator+(const ImVec2 &lhs, const ImVec2 &rhs) {
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}

static inline ImVec2 operator/(const ImVec2 &lhs, const float rhs) {
    return ImVec2(lhs.x / rhs, lhs.y / rhs);
}

static inline ImVec2 operator*(const ImVec2 &lhs, const float rhs) {
    return ImVec2(lhs.x * rhs, lhs.y * rhs);
}

ImVec2 ADIMainWindow::ImRotate(const ImVec2 &v, float cos_a, float sin_a) {
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}

void ADIMainWindow::ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size,
                                 float angle) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    ImVec2 _center =
        ((center * dpiScaleFactor) / 2.0f) + ImGui::GetCursorScreenPos();
    ImVec2 _size = size;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    ImVec2 pos[4] = {
        _center +
            ImRotate(ImVec2(-_size.x * 0.5f, -_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(_size.x * 0.5f, -_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(_size.x * 0.5f, +_size.y * 0.5f), cos_a, sin_a),
        _center +
            ImRotate(ImVec2(-_size.x * 0.5f, +_size.y * 0.5f), cos_a, sin_a)};
    ImVec2 uvs[4] = {ImVec2(0.0f, 0.0f), ImVec2(1.0f, 0.0f), ImVec2(1.0f, 1.0f),
                     ImVec2(0.0f, 1.0f)};

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0],
                            uvs[1], uvs[2], uvs[3], IM_COL32_WHITE);
}

void ADIMainWindow::CaptureDepthVideo() {
    if (view->depth_video_data_8bit != nullptr) {
        glBindTexture(GL_TEXTURE_2D, depth_video_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, view->frameWidth,
                     view->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     view->depth_video_data_8bit);
        glGenerateMipmap(GL_TEXTURE_2D);
        delete view->depth_video_data_8bit;

        ImVec2 _displayDepthDimensions = displayDepthDimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayDepthDimensions.x, _displayDepthDimensions.y);
        }

        ImageRotated(
            (ImTextureID)depth_video_texture,
            ImVec2(dictWinPosition["depth"][2], dictWinPosition["depth"][3]),
            ImVec2(_displayDepthDimensions.x, _displayDepthDimensions.y),
            rotationangleradians);
        //ImGui::Image((void*)(intptr_t)depth_video_texture, displayDepthDimensions);
    }
}

void ADIMainWindow::CaptureABVideo() {
    if (view->ab_video_data_8bit != nullptr) {
        glBindTexture(GL_TEXTURE_2D, ab_video_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, view->frameWidth,
                     view->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     view->ab_video_data_8bit);
        glGenerateMipmap(GL_TEXTURE_2D);
        delete view->ab_video_data_8bit;

        ImVec2 _displayABDimensions = displayABDimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayABDimensions.x, _displayABDimensions.y);
        }

        ImVec2 p = ImGui::GetCursorScreenPos();
        ImageRotated((ImTextureID)ab_video_texture,
                     ImVec2(dictWinPosition["ab"][2], dictWinPosition["ab"][3]),
                     ImVec2(_displayABDimensions.x, _displayABDimensions.y),
                     rotationangleradians);
        //ImGui::Image((void*)(intptr_t)ab_video_texture, displayABDimensions);
        size_t bb = 0;
    }
}

void ADIMainWindow::CapturePointCloudVideo() {
    //float currentFrame = glfwGetTime()/15;
    //deltaTime = currentFrame - lastFrame;
    //lastFrame = currentFrame;

    processInputs(window);

    preparePointCloudVertices(view->vertexBufferObject,
                              view->vertexArrayObject);

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPointSize(pointSize);

    // draw our Image
    glUseProgram(view->pcShader.Id());
    mat4x4_perspective(m_projection, radians(fov),
                       (float)view->frameWidth / (float)view->frameHeight, 0.1f,
                       100.0f);
    glUniformMatrix4fv(view->projectionIndex, 1, GL_FALSE, &m_projection[0][0]);

    //Look-At function[ x, y, z] = (cameraPos, cameraPos + cameraFront, cameraUp);
    vec3_add(cameraPos_Front, cameraPos, cameraFront);
    mat4x4_look_at(m_view, cameraPos, cameraPos_Front, cameraUp);
    glUniformMatrix4fv(view->viewIndex, 1, GL_FALSE, &m_view[0][0]);
    glUniformMatrix4fv(view->modelIndex, 1, GL_FALSE, &m_model[0][0]);

    glBindVertexArray(
        view->vertexArrayObject); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
    glDrawArrays(GL_POINTS, 0, view->vertexArraySize);
    glBindVertexArray(0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);

    ImVec2 _displayPointCloudDimensions = displayPointCloudDimensions;

    if (rotationangledegrees == 90 || rotationangledegrees == 270) {
        std::swap(_displayPointCloudDimensions.x,
                  _displayPointCloudDimensions.y);
    }

    ImVec2 p = ImGui::GetCursorScreenPos();
    ImageRotated(
        (ImTextureID)pointCloud_video_texture,
        ImVec2(dictWinPosition["pc"][2], dictWinPosition["pc"][3]),
        ImVec2(_displayPointCloudDimensions.x, _displayPointCloudDimensions.y),
        rotationangleradians);
    //ImGui::Image((void*)(intptr_t)pointCloud_video_texture, displayPointCloudDimensions);
    glDeleteVertexArrays(1, &view->vertexArrayObject);
    glDeleteBuffers(1, &view->vertexBufferObject);
}

void ADIMainWindow::pointCloudReset() {
    mat4x4_identity(m_view);
    mat4x4_identity(m_projection);
    mat4x4_identity(m_model);
    deltaTime = 0;
    lastFrame = 0;
    fov = 8.0f;
    yaw = -90.0f;
    pitch = 0.0f;
    view->Max_X = 6000.0;
    view->Max_Y = 6000.0;
    view->Max_Z = 6000.0;

    cameraPos[0] = 0.0f;
    cameraPos[1] = 0.0f;
    cameraPos[2] = 3.0f;
    cameraFront[0] = 0.0;
    cameraFront[1] = 0.0;
    cameraFront[2] = -1.0;
    cameraUp[0] = 0.0;
    cameraUp[1] = 1.0;
    cameraUp[2] = 0.0;
    pointSize = 1;
}

float ADIMainWindow::radians(float degrees) {
    float radians = (IM_PI / 180) * degrees;
    return radians;
}

void ADIMainWindow::processInputs(GLFWwindow *window) {
    //Sensitivity
    const float maxFov = 45.0f;
    float cameraSpeed = 2.5f * deltaTime;

    ImGuiIO &io = ImGui::GetIO(); //Get mouse events

    if (ImGui::IsWindowHovered()) {
        if (io.MouseWheel) //Use mouse wheel to scroll up/down for zoom in/zoom out
        {
            fov -= (float)io.MouseWheel;
            if (fov < 1.0f) {
                fov = 1.0f;
            }
            if (fov > maxFov) {
                fov = maxFov;
            }
        }
    }

    MouseMovementType movementType = MouseMovementType::None;

    ImVec2 mouseDownPos(-1.f, -1.f);
    //Press the left button for image rotation
    if (io.MouseDown[GLFW_MOUSE_BUTTON_1]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_1];
        movementType = MouseMovementType::Rotation;
        mouseDown = true;
    }
    //Press the right button for image translation
    else if (io.MouseDown[GLFW_MOUSE_BUTTON_2]) {
        mouseDownPos = io.MouseClickedPos[GLFW_MOUSE_BUTTON_2];
        movementType = MouseMovementType::Translation;
        mouseDown = true;
    } else {
        mouseDown = false;
    }

    if (mouseDown) {
        const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
        // Normalize to the image start coordinates
        //
        mouseDownPos.x -= imageStartPos.x;
        mouseDownPos.y -= imageStartPos.y;

        const vec2 mouseDelta{io.MouseDelta.x, io.MouseDelta.y};
        const vec2 displayDimensions{static_cast<float>(mainWindowWidth),
                                     static_cast<float>(mainWindowHeight)};
        // Only count drags if they originated on the image
        //
        if (mouseDownPos.x >= 0.f &&
            mouseDownPos.x <= displayPointCloudDimensions.x &&
            mouseDownPos.y >= 0.f &&
            mouseDownPos.y <= displayPointCloudDimensions.y) {
            if (movementType == MouseMovementType::Rotation) {
                const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
                const vec2 mousePos{io.MousePos.x + imageStartPos.x,
                                    io.MousePos.y + imageStartPos.y};
                vec2 lastMousePos;

                vec2_copy(lastMousePos, mousePos);
                vec2_sub(lastMousePos, mousePos, mouseDelta);

                quat newRotationQuat;
                GetArcballRotation(newRotationQuat, displayDimensions,
                                   lastMousePos, mousePos);

                mat4x4 newRotationMtx;
                mat4x4_from_quat(newRotationMtx, newRotationQuat);

                MatrixMultiply(m_model, newRotationMtx, m_model);
            }
            if (movementType == MouseMovementType::Translation) {
                cameraPos[0] -=
                    mouseDelta[0] * TranslationSensitivity / (maxFov - fov + 1);
                cameraPos[1] -=
                    mouseDelta[1] * TranslationSensitivity / (maxFov - fov + 1);
            }
        }
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, cameraFront, cameraSpeed);
        vec3_add(cameraPos, cameraPos, aux);
    }

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_scale(aux, cameraFront, cameraSpeed);
        vec3_sub(cameraPos, cameraPos, aux);
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, cameraFront, cameraUp);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_sub(cameraPos, cameraPos, aux);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        vec3 aux = {0.0, 0.0, 0.0};
        vec3_mul_cross(aux, cameraFront, cameraUp);
        vec3_norm(aux, aux);
        vec3_scale(aux, aux, cameraSpeed);
        vec3_add(cameraPos, cameraPos, aux);
    }
}

void ADIMainWindow::GetArcballRotation(quat rotation,
                                       const vec2 displayDimensions,
                                       const vec2 startPos, const vec2 endPos) {
    vec3 startVector;
    MapToArcball(startVector, displayDimensions, startPos);

    vec3 endVector;
    MapToArcball(endVector, displayDimensions, endPos);

    vec3 cross;
    vec3_mul_cross(cross, startVector, endVector);

    constexpr float epsilon = 0.001f;
    if (vec3_len(cross) < epsilon) {
        // Smooth out floating point error if the user didn't move the mouse
        // enough that it should register
        //
        quat_identity(rotation);
    } else {
        // The first 3 elements of the quaternion are the unit vector perpendicular
        // to the rotation (i.e. the cross product); the last element is the magnitude
        // of the rotation
        //
        vec3_copy(rotation, cross);
        vec3_norm(cross, cross);
        rotation[3] = vec3_mul_inner(startVector, endVector);
    }
}

void ADIMainWindow::MapToArcball(vec3 out, const vec2 displayDimensions,
                                 const vec2 mousePos) {
    // Scale coords to (-1, 1) to simplify some of the math
    //
    vec2 scaledMousePos;
    for (int i = 0; i < 2; ++i) {
        scaledMousePos[i] =
            mousePos[i] * (1.0f / ((displayDimensions[i] - 1.0f) * 0.5f)) -
            1.0f;
    }

    float lenSquared = scaledMousePos[0] * scaledMousePos[0] +
                       scaledMousePos[1] * scaledMousePos[1];

    // If the point is 'outside' our virtual sphere, we need to normalize to the sphere
    // This works because our sphere is of radius 1
    //
    if (lenSquared > 1.f) {
        const float normalizationFactor = 1.f / std::sqrt(lenSquared);

        // Return a point on the edge of the sphere
        //
        out[0] = scaledMousePos[0] * normalizationFactor;
        out[1] = scaledMousePos[1] * normalizationFactor;
        out[2] = 0.f;
    } else {
        // Return a point inside the sphere
        //
        out[0] = scaledMousePos[0];
        out[1] = scaledMousePos[1];
        out[2] = std::sqrt(1.f - lenSquared);
    }
}

void ADIMainWindow::MatrixMultiply(mat4x4 out, mat4x4 a, mat4x4 b) {
    mat4x4 atmp;
    mat4x4 btmp;
    mat4x4_dup(atmp, a);
    mat4x4_dup(btmp, b);
    mat4x4_mul(out, a, b);
}

void ADIMainWindow::computeFPS(int &fps) {
    frameCounter++;
    auto currentTime = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed = currentTime - startTime;
    if (elapsed.count() >= 2) {
        fps = frameCounter / (int)elapsed.count();
        frameCounter = 0;
        startTime = currentTime;
    }
}

bool ADIMainWindow::checkCameraSetToReceiveContent(
    const std::string &contentType) {
    aditof::CameraDetails camDetails;
    view->m_ctrl->m_cameras[0]->getDetails(camDetails);
    auto dataDetails = camDetails.frameType.dataDetails;
    auto it =
        std::find_if(dataDetails.begin(), dataDetails.end(),
                     [&contentType](const aditof::FrameDataDetails details) {
                         return details.type == contentType;
                     });
    return (it != dataDetails.end());
}

std::shared_ptr<aditof::Camera> ADIMainWindow::getActiveCamera() {
    if (!view || !view->m_ctrl || view->m_ctrl->m_cameras.empty() ||
        m_selectedDevice < 0 ||
        m_selectedDevice >= view->m_ctrl->m_cameras.size()) {
        return nullptr;
    }
    return view->m_ctrl->m_cameras[m_selectedDevice];
}

int ADIMainWindow::saveIniFile() {
    std::ofstream outputFile;
    auto currentTime = std::chrono::system_clock::now();
    // Convert the time point to a time_t (standard C time type)
    std::time_t currentTime_t =
        std::chrono::system_clock::to_time_t(currentTime);
    // Convert time_t to tm structure (for formatting)
    std::tm *timeInfo = std::localtime(&currentTime_t);
    // Format the time as a string
    char filename[100]; // Buffer to hold the formatted string
    std::strftime(filename, 100, "%Y-%m-%d-%H-%M-%S.ini", timeInfo);

    outputFile.open(filename);

    if (outputFile.is_open()) {
        outputFile << "abThreshMin=" << modified_ini_params["ab_thresh_min"]
                   << "\n";
        outputFile << "abSumThresh=" << modified_ini_params["ab_sum_thresh"]
                   << "\n";
        outputFile << "confThresh=" << modified_ini_params["conf_thresh"]
                   << "\n";
        outputFile << "radialThreshMin="
                   << modified_ini_params["radial_thresh_min"] << "\n";
        outputFile << "radialThreshMax="
                   << modified_ini_params["radial_thresh_max"] << "\n";
        outputFile << "jblfApplyFlag=" << modified_ini_params["jblf_apply_flag"]
                   << "\n";
        outputFile << "jblfWindowSize="
                   << modified_ini_params["jblf_window_size"] << "\n";
        outputFile << "jblfGaussianSigma="
                   << modified_ini_params["jblf_gaussian_sigma"] << "\n";
        outputFile << "jblfExponentialTerm="
                   << modified_ini_params["jblf_exponential_term"] << "\n";
        outputFile << "jblfMaxEdge=" << modified_ini_params["jblf_max_edge"]
                   << "\n";
        outputFile << "jblfABThreshold="
                   << modified_ini_params["jblf_ab_threshold"];

        outputFile.close();
        LOG(INFO) << "Modified parameters have been written to ini file: "
                  << filename;
    } else {
        LOG(ERROR) << "Unable to save ini parameters to file";
    }

    return 0;
}

void ADIMainWindow::iniParamWarn(std::string variable, std::string validVal) {
    ImGui::BeginPopupModal("Error", NULL, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Invalid %s value.", variable.c_str());
    ImGui::Text(validVal.c_str());
    ImGui::EndPopup();
}