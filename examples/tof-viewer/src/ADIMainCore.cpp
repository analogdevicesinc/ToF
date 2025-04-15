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
#include "aditof/status_definitions.h"
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

// TODO: These need to move to the class and not be global
uint8_t last_mode = -1;
std::map<std::string, std::string> ini_params;
std::map<std::string, std::string> modified_ini_params;
std::map<std::string, std::string> last_ini_params;
bool use_modified_ini_params = false;
char saveConfigurationPath[512] = "currentconfiguration.json";

ADIMainWindow::ADIMainWindow() : m_skipNetworkCameras(true) {
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
    char timebuff[100];
    time_t timeNow = time(0);
    struct tm *timeinfo;
    time(&timeNow);
    timeinfo = localtime(&timeNow);
    strftime(timebuff, sizeof(timebuff), "%Y%m%d_%H%M%S", timeinfo);
    // Concatenate the whole path
    strcpy_s(wholeLogPath, folderName);
#ifdef _WIN32
    strcat_s(wholeLogPath, "\\");
#elif __linux__
    strcat(wholeLogPath, "/");
#endif
    strcat_s(wholeLogPath, "log_");
    strcat_s(wholeLogPath, timebuff);
    strcat_s(wholeLogPath, ".txt");

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
        CameraStop();
    }

    //Recording flags
    if (view != nullptr && !view->m_ctrl->m_recorder->getFinishRecording()) {
        view->m_ctrl->m_recorder->stopRecording();
    }
    if (view != nullptr && !view->m_ctrl->m_recorder->getStopPlayback()) {
        view->m_ctrl->m_recorder->stopPlayback();
        CameraStop();
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

void ADIMainWindow::CustomizeMenus() {
    ImGuiStyle &style = ImGui::GetStyle();

    // Set the color of the border
    style.Colors[ImGuiCol_Border] =
        ImVec4(0.7f, 0.7f, 0.7f, 1.0f); // (R, G, B, A);
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
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only

    std::string version = aditof::getApiVersion();
    std::string _title = "Analog Devices, Inc. Time of Flight Main Window v" +
                         version; //Default name

    // Create window with graphics context
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    window = glfwCreateWindow(1280, 1024, _title.c_str(), NULL, NULL);

    if (window == NULL) {
        return false;
    }

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
    //ImGui::StyleColorsDark();
    //OR
    ImGui::StyleColorsClassic();
    CustomizeMenus();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    RefreshDevices();

    /**************/

    return true;
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

std::shared_ptr<aditof::Camera> ADIMainWindow::getActiveCamera() {
    if (!view || !view->m_ctrl || view->m_ctrl->m_cameras.empty() ||
        m_selectedDevice < 0 ||
        m_selectedDevice >= view->m_ctrl->m_cameras.size()) {
        return nullptr;
    }
    return view->m_ctrl->m_cameras[m_selectedDevice];
}

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
        if (!m_callbackInitialized) {
            handleInterruptCallback();
            m_callbackInitialized = true;
        }
        /***************************************************/
        //Create windows here:
        showMainMenu();
        if (isPlaying) {
            CameraPlay(modeSelection, viewSelection);
            computeFPS(m_fps);
            if (view->m_ctrl->panicStop) {
                CameraStop();

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

void ADIMainWindow::showMainMenu() {
    static bool show_app_log = true;
    static bool show_ini_window = false;

    if (show_app_log) {
        showLogWindow(&show_app_log);
    }

    if (show_ini_window && isPlaying) {
        showIniWindow(&show_ini_window);
    }

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
            ImGui::MenuItem("Debug Log", nullptr, &show_app_log);
            ImGui::MenuItem("Ini Params", nullptr, &show_ini_window,
                            m_cameraWorkerDone && isPlaying);
            ImGui::Separator();
            if (ImGui::MenuItem("Load Configuration", nullptr, false,
                                m_cameraWorkerDone && !isPlaying)) {
                showLoadAdsdParamsMenu();
            }
            if (ImGui::MenuItem("Save Configuration", nullptr, false,
                                m_cameraWorkerDone)) {
                showSaveAdsdParamsMenu();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void ADIMainWindow::showRecordMenu() {
    if (ImGui::BeginMenu("Record Options", m_cameraWorkerDone)) {
        ImGui::InputInt("Seconds to Record", &recordingSeconds, 1, MAX_RECORD_TIME);

        if (recordingSeconds < 1) {
            recordingSeconds = 1;
        } else if (recordingSeconds > MAX_RECORD_TIME) {
            recordingSeconds = MAX_RECORD_TIME;
        }

        ImGui::NewLine();

        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
            ImGuiExtensions::ButtonColorChanger colorChangerStartRec(
                customColorPlay, isPlaying);
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
                strftime(time_buffer, sizeof(time_buffer), "%Y%m%d%H%M",
                         &timeinfo);
                tempPath += "\\mode_" + std::to_string(modeSelection) +
                            "_frames" + std::string(time_buffer);
                int filterIndex = 0;
                char tempbuff[MAX_PATH];
                tempPath.copy(tempbuff, tempPath.length(), 0);
                tempbuff[tempPath.length()] = '\0';
                std::string saveFile = getADIFileName(
                    nullptr, "ADI ToF Recording Files\0*.bin\0All Files\0*.*\0",
                    tempbuff, filterIndex);
                //Check if filename exists and format is corrct
                if (!saveFile.empty() && filterIndex) {
                    if (!isPlaying) {
                        //"Press" the play button, in case it is not pressed.
                        CameraPlay(
                            modeSelection,
                            viewSelection); //Which ever is currently selected
                        isPlaying = true;
                    }
                    //setting binary save option
                    view->m_ctrl->m_saveBinaryFormat =
                        view->getSaveBinaryFormat();
                    view->m_ctrl->startRecording(saveFile, view->frameHeight,
                                                 view->frameWidth,
                                                 recordingSeconds);
                    isRecording = true;
                }
            }
        }

        ImGui::SameLine();
        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
            ImGuiExtensions::ButtonColorChanger colorChangerRecStop(
                customColorStop, isRecording);
            if (ImGuiExtensions::ADIButton("Stop Recording", isRecording)) {
                view->m_ctrl->stopRecording();
                isRecording = false;
                isPlayRecorded = false;
                CameraStop(); //TODO: Create a Stop ToF camera
                isPlaying = false;
            }
        }
        ImGui::EndMenu();
    }
}

void ADIMainWindow::showLoadAdsdParamsMenu() {

    int FilterIndex = 0;
    std::string fs =
        openADIFileName("ADI ToF Config Files\0*.json\0", nullptr, FilterIndex);
    LOG(INFO) << "Load File selected: " << fs;

    if (fs.empty()) {
        return;
    }

    bool loadconfigurationFile = false;
    std::string loadconfigurationFileValue = std::string(fs);
    if (!loadconfigurationFileValue.empty()) {
        if (loadconfigurationFileValue.find(".json") == std::string::npos) {
            loadconfigurationFileValue += ".json";
        }
        loadconfigurationFile = true;
    }
    if (loadconfigurationFile && view) {
        auto camera = view->m_ctrl->m_cameras[static_cast<unsigned int>(
            view->m_ctrl->getCameraInUse())];

        aditof::Status status =
            camera->loadDepthParamsFromJsonFile(loadconfigurationFileValue);

        if (status != aditof::Status::OK) {
            LOG(INFO) << "Could not load current configuration "
                         "info to "
                      << loadconfigurationFileValue;
        } else {
            LOG(INFO) << "Current configuration info from file "
                      << loadconfigurationFileValue;
        }
    }
}

void ADIMainWindow::showSaveAdsdParamsMenu() {

    ImGuiExtensions::ButtonColorChanger colorChangerStartRec(customColorPlay,
                                                             isPlaying);

    char filename[MAX_PATH] = "";
    int FilterIndex;
    std::string fs = getADIFileName(
        nullptr, "ADI ToF Config Files\0*.json\0All Files\0*.*\0", filename,
        FilterIndex);
    LOG(INFO) << "Selecting to save configuration the file: " << fs;

    bool saveconfigurationFile = false;
    std::string saveconfigurationFileValue = fs;

    if (!saveconfigurationFileValue.empty()) {
        if (saveconfigurationFileValue.find(".json") == std::string::npos) {
            saveconfigurationFileValue += ".json";
        }
        saveconfigurationFile = true;
    }
    if (saveconfigurationFile && view) {
        auto camera = view->m_ctrl->m_cameras[static_cast<unsigned int>(
            view->m_ctrl->getCameraInUse())];

        aditof::Status status =
            camera->saveDepthParamsToJsonFile(saveconfigurationFileValue);

        if (status != aditof::Status::OK) {
            saveconfigurationFile = false;
            LOG(INFO) << "Could not save current configuration info to "
                      << saveconfigurationFileValue << std::endl;
        } else {
            LOG(INFO) << "Current configuration info saved to file "
                      << saveconfigurationFileValue << std::endl;
            strcpy_s(saveConfigurationPath,
                        saveconfigurationFileValue.c_str());
        }
    }
}

void ADIMainWindow::showDeviceMenu() {
    if (!isPlaying && !isPlayRecorded) {
        if (ImGui::BeginMenu("Device")) {
            ImGuiExtensions::ADIComboBox(
                "Device", "(No available devices)", ImGuiComboFlags_None,
                m_connectedDevices, &m_selectedDevice, _isOpenDevice);
            //If a device is found, then set the first one found
            if (!m_connectedDevices.empty() && m_selectedDevice == -1) {
                m_selectedDevice = 0;
                _isOpenDevice = true;
            }

            bool _noConnected = m_connectedDevices.empty();
            if (ImGuiExtensions::ADIButton("Refresh", _noConnected)) {
                _isOpenDevice = false;
                m_cameraWorkerDone = false;
                RefreshDevices();
            }

            ImGui::SameLine();

            const bool openAvailable = !m_connectedDevices.empty();
            {

                { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
                    ImGuiExtensions::ButtonColorChanger colorChanger(
                        ImGuiExtensions::ButtonColor::Green, openAvailable);
                    if (ImGuiExtensions::ADIButton("Open", _isOpenDevice) &&
                        0 <= m_selectedDevice) {
                        if (isPlayRecorded) {
                            stopPlayback();
                        }

                        _isOpenDevice = false;
                        initCameraWorker = std::thread(
                            std::bind(&ADIMainWindow::InitCamera, this));
                    }
                }
            }
            ImGui::SameLine();
            if (ImGuiExtensions::ADIButton("Close", !_isOpenDevice)) {
                stopPlayback();
                CameraStop();
                m_cameraWorkerDone = false;
                m_callbackInitialized = false;
                m_cameraModes.clear();
                _cameraModes.clear();
                if (initCameraWorker.joinable()) {
                    initCameraWorker.join();
                }
                view.reset();
                RefreshDevices();
            }

            if (ImGuiExtensions::ADICheckbox("Max FPS Network Test (Debug)",
                                             &m_netLinkTest, _isOpenDevice)) {
                if (m_netLinkTest) {
                    m_ipSuffix = ":netlinktest";
                } else {
                    m_ipSuffix.clear();
                }
                RefreshDevices();
            }

            ImGui::EndMenu();
        }
    }

    if (m_cameraWorkerDone) {
        _isOpenDevice = false;
        if (!isPlaying && !isPlayRecorded) {
            if (ImGui::BeginMenu("ToF Camera Options")) {
                ImGui::Text("Mode:");
                ImGuiExtensions::ADIComboBox(
                    "", "Select Mode", ImGuiSelectableFlags_None,
                    m_cameraModesDropDown, &modeSelection, true);

                ImGui::NewLine();
                ImGui::Text("View Options:");
                ImGuiExtensions::ADIRadioButton("Active Brightness and Depth",
                                                &viewSelection, 0);
                ImGuiExtensions::ADIRadioButton("Point Cloud and Depth",
                                                &viewSelection, 1);
                ImGui::NewLine();
                ImGui::Text("Video:");

                { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
                    ImGuiExtensions::ButtonColorChanger colorChangerPlay(
                        customColorPlay, !isPlaying);
                    if (ImGuiExtensions::ADIButton(
                            "Play", !isPlaying && !isPlayRecorded)) {
                        viewSelectionChanged = viewSelection;
                        isPlaying = true;
                        ini_params.clear();
                    }
                }
                ImGui::SameLine();
                { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
                    ImGuiExtensions::ButtonColorChanger colorChangerStop(
                        customColorStop, isPlaying);
                    if (ImGuiExtensions::ADIButton("Stop", isPlaying)) {
                        isPlaying = false;
                        isPlayRecorded = false;
                        m_fps_frameRecvd = 0;
                        CameraStop();
                        if (isRecording) {
                            view->m_ctrl->stopRecording();
                            isRecording = false;
                        }
                    }
                }
                ImGui::EndMenu();
            }
        }

        if (view != NULL && view->m_ctrl->recordingFinished() && isRecording) {
            view->m_ctrl->stopRecording();
            isRecording = false;
        }
    }
}

void ADIMainWindow::showPlaybackMenu() {
    /**********/
    //Playback
    if (ImGui::BeginMenu("Playback Options")) {
        float customColorOpenRec = 0.22f;
        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
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
                            std::make_shared<adicontroller::ADIController>(
                                std::vector<std::shared_ptr<aditof::Camera>>(
                                    m_camerasList)),
                            "Record Viewer");
                    }

                    view->m_ctrl->startPlayback(path);
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
                    LOG(ERROR)
                        << "Non-existing file or unsupported file format "
                           "chosen for playback";
                }
            }
        }

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
    CameraStop();
    //view.reset();
    LOG(INFO) << "Stream has been stopped.";
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
        DisplayActiveBrightnessWindow(overlayFlags);
    }
    if (displayDepth) {
        DisplayDepthWindow(overlayFlags);
    }
    if (displayPointCloud && viewSelection == 1) {
        DisplayPointCloudWindow(overlayFlags);
    }
    if (displayTemp) {
        DisplayInfoWindow(overlayFlags);
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
        status = getActiveCamera()->getFrameProcessParams(ini_params);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Could not get ini params";
        } else {
            abThreshMin = std::stof(ini_params["abThreshMin"]);
            abSumThresh = std::stof(ini_params["abThreshMin"]);
            confThresh = std::stof(ini_params["confThresh"]);
            radialThreshMin = std::stof(ini_params["radialThreshMin"]);
            radialThreshMax = std::stof(ini_params["radialThreshMax"]);
            if (static_cast<int>(
                    std::round(std::stof(ini_params["jblfApplyFlag"]))) == 1) {
                jblfApplyFlag = true;
            } else {
                jblfApplyFlag = false;
            }

            jblfWindowSize = static_cast<int>(
                std::round(std::stof(ini_params["jblfWindowSize"])));
            jblfGaussianSigma = std::stof(ini_params["jblfGaussianSigma"]);
            jblfExponentialTerm = std::stof(ini_params["jblfExponentialTerm"]);
            jblfMaxEdge = std::stof(ini_params["jblfMaxEdge"]);
            jblfABThreshold = std::stof(ini_params["jblfABThreshold"]);
            headerSize = static_cast<int>(
                std::round(std::stof(ini_params["headerSize"])));
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
            if (last_ini_params["abThreshMin"] != std::to_string(abThreshMin)) {
                LOG(ERROR)
                    << "Invalid abThreshMin value. Valid values [0 - 65535]";
                last_ini_params["abThreshMin"] = std::to_string(abThreshMin);
            }
            iniParamWarn("abThreshMin", "Valid value: [0 - 65535]");
        }
        ImGui::InputFloat("abSumThresh", &abSumThresh);
        ImGui::InputFloat("confThresh", &confThresh);
        if (confThresh < 0 || confThresh > 255) {
            if (last_ini_params["confThresh"] != std::to_string(confThresh)) {
                LOG(ERROR)
                    << "Invalid confThresh value. Valid values [0 - 255]";
                last_ini_params["confThresh"] = std::to_string(confThresh);
            }

            iniParamWarn("confThresh", "Valid value: [0 - 255]");
        }
        ImGui::InputFloat("radialThreshMin", &radialThreshMin);
        if (radialThreshMin < 0 || radialThreshMin > 65535) {
            if (last_ini_params["radialThreshMin"] !=
                std::to_string(radialThreshMin)) {
                LOG(ERROR) << "Invalid radialThreshMin value. Valid values [0 "
                              "- 65535]";
                last_ini_params["radialThreshMin"] =
                    std::to_string(radialThreshMin);
            }
            iniParamWarn("radialThreshMin", "Valid value:[0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            if (last_ini_params["radialThreshMin"] !=
                std::to_string(radialThreshMin)) {
                LOG(ERROR)
                    << "radialThreshMin should be less than radialThreshMax";
                last_ini_params["radialThreshMin"] =
                    std::to_string(radialThreshMin);
            }
            iniParamWarn(
                "radialThreshMin",
                "radialThreshMin should be\nless than radialThreshMax");
        }
        ImGui::InputFloat("radialThreshMax", &radialThreshMax);
        if (radialThreshMax < 0 || radialThreshMax > 65535) {
            if (last_ini_params["radialThreshMax"] !=
                std::to_string(radialThreshMax)) {
                LOG(ERROR) << "Invalid radialThreshMax value. Valid values [0 "
                              "- 65535]";
                last_ini_params["radialThreshMax"] =
                    std::to_string(radialThreshMax);
            }
            iniParamWarn("radialThreshMax", "Valid values [0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            if (last_ini_params["radialThreshMax"] !=
                std::to_string(radialThreshMax)) {
                LOG(ERROR)
                    << "radialThreshMax should be greater than radialThreshMin";
                last_ini_params["radialThreshMax"] =
                    std::to_string(radialThreshMax);
            }
            iniParamWarn(
                "radialThreshMax",
                "radialThreshMax should be\ngreater than radialThreshMin");
        }
        ImGui::Checkbox("jblfApplyFlag", &jblfApplyFlag);
        ImGui::InputInt("jblfWindowSize", &jblfWindowSize);
        if (jblfWindowSize != 3 && jblfWindowSize != 5 && jblfWindowSize != 7) {
            if (last_ini_params["jblfWindowSize"] !=
                std::to_string(jblfWindowSize)) {
                LOG(ERROR)
                    << "Invalid jblfWindowSize value. Valid values [3, 5, 7]";
                last_ini_params["jblfWindowSize"] =
                    std::to_string(jblfWindowSize);
            }

            iniParamWarn("jblfWindowSize", "Valid value: [3, 5, 7]");
        }
        ImGui::InputFloat("jblfGaussianSigma", &jblfGaussianSigma);
        if (jblfGaussianSigma < 0 || jblfGaussianSigma > 65535) {
            if (last_ini_params["jblfGaussianSigma"] !=
                std::to_string(jblfGaussianSigma)) {
                LOG(ERROR) << "Invalid jblfGaussianSigma value. Valid values "
                              "[0 - 65535]";
                last_ini_params["jblfGaussianSigma"] =
                    std::to_string(jblfGaussianSigma);
            }
            iniParamWarn("jblfGaussianSigma", "Valid value: [0 - 65535]");
        }
        ImGui::InputFloat("jblfExponentialTerm", &jblfExponentialTerm);
        if (jblfExponentialTerm < 0 || jblfExponentialTerm > 255) {
            if (last_ini_params["jblfExponentialTerm"] !=
                std::to_string(jblfExponentialTerm)) {
                LOG(ERROR)
                    << "Invalid jblfExponentialTerm value. Valid values [0 "
                       "- 255]";
                last_ini_params["jblfExponentialTerm"] =
                    std::to_string(jblfExponentialTerm);
            }
            iniParamWarn("jblfExponentialTerm", "Valid value: [0 - 255]");
        }
        ImGui::InputFloat("jblfMaxEdge", &jblfMaxEdge);
        if (jblfMaxEdge < 0 || jblfMaxEdge > 63) {
            if (last_ini_params["jblfMaxEdge"] != std::to_string(jblfMaxEdge)) {
                LOG(ERROR) << "Invalid jblfMaxEdge value. Valid values [0 "
                              "- 63]";
                last_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
            }
            iniParamWarn("jblfMaxEdge", "Valid value: [0 - 63]");
        }
        ImGui::InputFloat("jblfABThreshold", &jblfABThreshold);
        if (jblfABThreshold < 0 || jblfABThreshold > 131071) {
            if (last_ini_params["jblfABThreshold"] !=
                std::to_string(jblfABThreshold)) {
                LOG(ERROR) << "Invalid jblfABThreshold value. Valid values [0 "
                              "- 131071]";
                last_ini_params["jblfABThreshold"] =
                    std::to_string(jblfABThreshold);
            }
            iniParamWarn("jblfABThreshold", "Valid value: [0 - 131071]");
        }
        ImGui::Checkbox("Metadata Over AB", &metadata);

        // modify ini params
        modified_ini_params["abThreshMin"] = std::to_string(abThreshMin);
        modified_ini_params["abSumThresh"] = std::to_string(abSumThresh);
        modified_ini_params["confThresh"] = std::to_string(confThresh);
        modified_ini_params["radialThreshMin"] =
            std::to_string(radialThreshMin);
        modified_ini_params["radialThreshMax"] =
            std::to_string(radialThreshMax);
        if (jblfApplyFlag) {
            modified_ini_params["jblfApplyFlag"] = std::to_string(1);
        } else {
            modified_ini_params["jblfApplyFlag"] = std::to_string(0);
        }
        modified_ini_params["jblfWindowSize"] = std::to_string(jblfWindowSize);
        modified_ini_params["jblfGaussianSigma"] =
            std::to_string(jblfGaussianSigma);
        modified_ini_params["jblfExponentialTerm"] =
            std::to_string(jblfExponentialTerm);
        modified_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
        modified_ini_params["jblfABThreshold"] =
            std::to_string(jblfABThreshold);

        // keep ini param input from last time to skip repetitve error log messages
        last_ini_params["abThreshMin"] = std::to_string(abThreshMin);
        last_ini_params["confThresh"] = std::to_string(confThresh);
        last_ini_params["radialThreshMin"] = std::to_string(radialThreshMin);
        last_ini_params["radialThreshMax"] = std::to_string(radialThreshMax);
        last_ini_params["jblfWindowSize"] = std::to_string(jblfWindowSize);
        last_ini_params["jblfGaussianSigma"] =
            std::to_string(jblfGaussianSigma);
        last_ini_params["jblfExponentialTerm"] =
            std::to_string(jblfExponentialTerm);
        last_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
        last_ini_params["jblfABThreshold"] = std::to_string(jblfABThreshold);

        if (ImGui::Button("Modify")) {
            // stop streaming
            isPlaying = false;
            isPlayRecorded = false;
            m_fps_firstFrame = 0;
            m_fps_frameRecvd = 0;
            CameraStop();
            if (isRecording) {
                view->m_ctrl->stopRecording();
                isRecording = false;
            }

            use_modified_ini_params = true;
            
            // restart streaming
            viewSelectionChanged = viewSelection;
            isPlaying = true;
        }
    }
    ImGui::End();
}
void ADIMainWindow::iniParamWarn(std::string variable, std::string validVal) {
    ImGui::Text("Invalid %s value.", variable.c_str());
    ImGui::Text(validVal.c_str());
}