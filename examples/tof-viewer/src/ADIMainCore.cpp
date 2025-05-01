/********************************************************************************/
/*                                                                              */
/*  Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*   Portions Copyright (c) 2020 Analog Devices Inc.							*/
/*  Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/

#include "glad/gl.h"
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
#include <direct.h>
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

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

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

ADIMainWindow::ADIMainWindow() : m_skip_network_cameras(true) {
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
    m_file_stream = freopen(wholeLogPath, "w", stderr); // Added missing pointer
    setvbuf(m_file_stream, 0, _IONBF, 0);               // No Buffering
    m_file_input = fopen(wholeLogPath, "r");

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
                m_skip_network_cameras = true;
            } else if (value == "off") {
                m_skip_network_cameras = false;
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

    if (m_is_playing) {
        CameraStop();
    }

    // imGUI disposing
    //ImGui::GetIO().IniFilename = NULL;
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

bool ADIMainWindow::StartImGUI(const ADIViewerArgs &args) {
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
    bool err = gladLoadGL((GLADloadfunc)glfwGetProcAddress) == 0;
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

    if (args.HighDpi && 0) {
        m_dpi_scale_factor = HIGHDPISCALAR;
        m_is_high_dpi = true;
    } else {
        m_dpi_scale_factor = NORMALDPISCALAR;
        m_is_high_dpi = false;
    }
    SetDpi();

    m_dict_win_position["info"].x = 5.0f;
    m_dict_win_position["info"].y = 25.0f;
    m_dict_win_position["info"].width = 300.0f;
    m_dict_win_position["info"].height = 200.0f;

    m_dict_win_position["control"].x = m_dict_win_position["info"].x;
    m_dict_win_position["control"].y = WindowCalcY(m_dict_win_position["info"], 10.0f);
    m_dict_win_position["control"].width = m_dict_win_position["info"].width;
    m_dict_win_position["control"].height = 400.0f;

    m_dict_win_position["fr-main"].x =
        WindowCalcX(m_dict_win_position["info"], 10.0f);
    m_dict_win_position["fr-main"].y = m_dict_win_position["info"].y;
    m_dict_win_position["fr-main"].width = 640.0f;
    m_dict_win_position["fr-main"].height = 640.0f;

    m_dict_win_position["fr-sub1"].x =
        WindowCalcX(m_dict_win_position["fr-main"], 10.0f);
    m_dict_win_position["fr-sub1"].y = m_dict_win_position["fr-main"].y;
    m_dict_win_position["fr-sub1"].width = 256.0f;
    m_dict_win_position["fr-sub1"].height = 256.0f;

    m_dict_win_position["fr-sub2"].x = m_dict_win_position["fr-sub1"].x;
    m_dict_win_position["fr-sub2"].y =
        WindowCalcY(m_dict_win_position["fr-sub1"], 10.0f);
    m_dict_win_position["fr-sub2"].width = 256.0f;
    m_dict_win_position["fr-sub2"].height = 256.0f;

    m_pc_position = &m_dict_win_position["fr-main"];
    m_ab_position = &m_dict_win_position["fr-sub1"];
    m_depth_position = &m_dict_win_position["fr-sub2"];

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

void ADIMainWindow::OpenGLCleanUp() {
    glDeleteTextures(1, &m_gl_ab_video_texture);
    glDeleteTextures(1, &m_gl_depth_video_texture);
    glDeleteTextures(1, &m_gl_pointcloud_video_texture);
    glDeleteVertexArrays(1, &m_view_instance->vertexArrayObject);
    glDeleteBuffers(1, &m_view_instance->vertexBufferObject);
    glDeleteProgram(m_view_instance->pcShader.Id());
    m_view_instance->pcShader.RemoveShaders();
}

void ADIMainWindow::SetDpi() {
    ImGui::GetStyle().ScaleAllSizes(m_dpi_scale_factor);

    // ImGui doesn't automatically scale fonts, so we have to do that ourselves
    //
    ImFontConfig fontConfig;
    constexpr float defaultFontSize = 13.0f;
    fontConfig.SizePixels = defaultFontSize * m_dpi_scale_factor;
    ImGui::GetIO().Fonts->AddFontDefault(&fontConfig);

    glfwGetWindowSize(window, &m_main_window_width, &m_main_window_height);
    m_main_window_width = static_cast<int>(m_main_window_width * m_dpi_scale_factor);
    m_main_window_height = static_cast<int>(m_main_window_height * m_dpi_scale_factor);
    glfwSetWindowSize(window, m_main_window_width, m_main_window_height);
}

void ADIMainWindow::SetWindowPosition(float x, float y) {
    x = x * m_dpi_scale_factor;
    y = y * m_dpi_scale_factor;
    ImVec2 winPos = {x, y};
    ImGui::SetNextWindowPos(winPos);
}

void ADIMainWindow::SetWindowSize(float width, float height) {
    width = width * m_dpi_scale_factor;
    height = height * m_dpi_scale_factor;
    ImVec2 m_size = {width, height};
    ImGui::SetNextWindowSize(m_size);
}

std::shared_ptr<aditof::Camera> ADIMainWindow::GetActiveCamera() {
    if (!m_view_instance || !m_view_instance->m_ctrl || m_view_instance->m_ctrl->m_cameras.empty() ||
        m_selected_device_index < 0 ||
        m_selected_device_index >= m_view_instance->m_ctrl->m_cameras.size()) {
        return nullptr;
    }
    return m_view_instance->m_ctrl->m_cameras[m_selected_device_index];
}

void ADIMainWindow::Render() {
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
        glfwGetWindowSize(window, &m_main_window_width, &m_main_window_height);
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        if (!m_callback_initialized) {
            HandleInterruptCallback();
            m_callback_initialized = true;
        }
        /***************************************************/
        //Create windows here:
        ShowMainMenu();
        if (m_is_playing) {
            CameraPlay(m_mode_selection, m_view_selection);
            ComputeFPS(m_fps);
            if (m_view_instance != nullptr) {
                if (m_view_instance->m_ctrl->panicStop) {
                    CameraStop();

                    aditof::Status status = aditof::Status::OK;
                    auto camera =
                        GetActiveCamera(); //already initialized on constructor

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
            }

        }
        else {
            // Show Start Wizard
            ShowStartWizard();
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

void ADIMainWindow::ShowMainMenu() {
    static bool show_app_log = true;
    static bool show_ini_window = false;

    if (show_app_log) {
        ShowLogWindow(&show_app_log);
    }

    if (show_ini_window && m_is_playing) {
        ShowIniWindow(&show_ini_window);
    }

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("?")) {
            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(window, true);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools")) {

            ImGui::MenuItem("Debug Log", nullptr, &show_app_log);
            ImGui::MenuItem("Ini Params", nullptr, &show_ini_window,
                            m_cameraWorkerDone && m_is_playing);
            ImGui::Separator();
            if (ImGui::MenuItem("Load Configuration", nullptr, false,
                                m_cameraWorkerDone && !m_is_playing)) {
                ShowLoadAdsdParamsMenu();
            }
            if (ImGui::MenuItem("Save Configuration", nullptr, false,
                                m_cameraWorkerDone)) {
                ShowSaveAdsdParamsMenu();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void ADIMainWindow::ShowLoadAdsdParamsMenu() {

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
    if (loadconfigurationFile && m_view_instance) {
        auto camera = m_view_instance->m_ctrl->m_cameras[static_cast<unsigned int>(
            m_view_instance->m_ctrl->getCameraInUse())];

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

void ADIMainWindow::ShowSaveAdsdParamsMenu() {

    ImGuiExtensions::ButtonColorChanger colorChangerStartRec(m_custom_color_play,
                                                             m_is_playing);

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
    if (saveconfigurationFile && m_view_instance) {
        auto camera = m_view_instance->m_ctrl->m_cameras[static_cast<unsigned int>(
            m_view_instance->m_ctrl->getCameraInUse())];

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

void ADIMainWindow::StopPlayback() {
    m_is_playing = false;
    m_is_open_device = true;
    CameraStop();
    //view.reset();
    LOG(INFO) << "Stream has been stopped.";
}

void ADIMainWindow::ShowIniWindow(bool *p_open) {
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

    if (m_is_playing && ini_params.empty()) {
        status = GetActiveCamera()->getFrameProcessParams(ini_params);
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
        ImGui::PushItemWidth(140 * m_dpi_scale_factor);
        ImGui::InputFloat("abThreshMin", &abThreshMin);
        if (abThreshMin < 0 || abThreshMin > 65535) {
            if (last_ini_params["abThreshMin"] != std::to_string(abThreshMin)) {
                LOG(ERROR)
                    << "Invalid abThreshMin value. Valid values [0 - 65535]";
                last_ini_params["abThreshMin"] = std::to_string(abThreshMin);
            }
            IniParamWarn("abThreshMin", "Valid value: [0 - 65535]");
        }
        ImGui::InputFloat("abSumThresh", &abSumThresh);
        ImGui::InputFloat("confThresh", &confThresh);
        if (confThresh < 0 || confThresh > 255) {
            if (last_ini_params["confThresh"] != std::to_string(confThresh)) {
                LOG(ERROR)
                    << "Invalid confThresh value. Valid values [0 - 255]";
                last_ini_params["confThresh"] = std::to_string(confThresh);
            }

            IniParamWarn("confThresh", "Valid value: [0 - 255]");
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
            IniParamWarn("radialThreshMin", "Valid value:[0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            if (last_ini_params["radialThreshMin"] !=
                std::to_string(radialThreshMin)) {
                LOG(ERROR)
                    << "radialThreshMin should be less than radialThreshMax";
                last_ini_params["radialThreshMin"] =
                    std::to_string(radialThreshMin);
            }
            IniParamWarn(
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
            IniParamWarn("radialThreshMax", "Valid values [0 - 65535]");
        }
        if (radialThreshMin >= radialThreshMax) {
            if (last_ini_params["radialThreshMax"] !=
                std::to_string(radialThreshMax)) {
                LOG(ERROR)
                    << "radialThreshMax should be greater than radialThreshMin";
                last_ini_params["radialThreshMax"] =
                    std::to_string(radialThreshMax);
            }
            IniParamWarn(
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

            IniParamWarn("jblfWindowSize", "Valid value: [3, 5, 7]");
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
            IniParamWarn("jblfGaussianSigma", "Valid value: [0 - 65535]");
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
            IniParamWarn("jblfExponentialTerm", "Valid value: [0 - 255]");
        }
        ImGui::InputFloat("jblfMaxEdge", &jblfMaxEdge);
        if (jblfMaxEdge < 0 || jblfMaxEdge > 63) {
            if (last_ini_params["jblfMaxEdge"] != std::to_string(jblfMaxEdge)) {
                LOG(ERROR) << "Invalid jblfMaxEdge value. Valid values [0 "
                              "- 63]";
                last_ini_params["jblfMaxEdge"] = std::to_string(jblfMaxEdge);
            }
            IniParamWarn("jblfMaxEdge", "Valid value: [0 - 63]");
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
            IniParamWarn("jblfABThreshold", "Valid value: [0 - 131071]");
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
            m_is_playing = false;
            m_fps_first_frame_number = 0;
            m_fps_frame_received = 0;
            CameraStop();

            use_modified_ini_params = true;
            
            // restart streaming
            m_view_selection_changed = m_view_selection;
            m_is_playing = true;
        }
    }
    ImGui::End();
}
void ADIMainWindow::IniParamWarn(std::string variable, std::string validVal) {
    ImGui::Text("Invalid %s value.", variable.c_str());
    ImGui::Text(validVal.c_str());
}

void ADIMainWindow::DrawColoredLabel(const char *fmt, ...) {
    // Format the text
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ImVec4 box_color = ImVec4(0.2f, 0.6f, 0.9f, 1.0f); // RGBa

    // Get draw list and cursor position
    ImVec2 textPos = ImGui::GetCursorScreenPos();
    ImVec2 textSize = ImGui::CalcTextSize(buf);
    float padding = 2.0f;

    // Draw background box
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(ImVec2(textPos.x - padding, textPos.y - padding),
                            ImVec2(textPos.x + textSize.x + padding,
                                   textPos.y + textSize.y + padding),
                            ImGui::ColorConvertFloat4ToU32(box_color),
                            4.0f // optional: corner radius
    );

    // Draw the text on top
    ImGui::SetCursorScreenPos(textPos);
    ImGui::TextUnformatted(buf);
}

void ADIMainWindow::DrawBarLabel(const char *fmt, ...) {

    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    ImGui::PushStyleColor(
        ImGuiCol_ChildBg,
        IM_COL32(60, 60, 60, 255)); // Optional: dark bar background

    float textHeight = ImGui::GetTextLineHeight();
    ImGui::BeginChild(buf, ImVec2(0, textHeight * 1.1f), false);

    // Center the text
    float windowWidth = ImGui::GetWindowSize().x;
    float textWidth = ImGui::CalcTextSize(buf).x;
    ImGui::SetCursorPosX((windowWidth - textWidth) *
                         0.5f); // center horizontally
    ImGui::Text(buf);

    ImGui::EndChild();

    ImGui::PopStyleColor(); // Reset bar color
}

void ADIMainWindow::NewLine(float spacing) { ImGui::Dummy(ImVec2(0.0f, spacing)); }

void ADIMainWindow::ShowStartWizard() {

    ImGuiIO &io = ImGui::GetIO(); // Get the display size
    ImVec2 center = ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.2f);
    ImVec2 window_size = ImVec2(400, 320); // Your window size

    // Offset to truly center it
    ImVec2 window_pos = ImVec2(center.x - window_size.x * 0.5f,
                               center.y - window_size.y * 0.5f);

    // Set position and size before Begin()
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(window_size, ImGuiCond_Always);

    ImGui::Begin("Camera Selection Wizard", nullptr, ImGuiWindowFlags_NoResize);

    static uint32_t selected = 1;

    ImGui::RadioButton("Saved Stream", selected == 0);
    if (ImGui::IsItemClicked())
        selected = 0;
    ImGui::SameLine();
    ImGui::RadioButton("Live Camera", selected == 1);
    if (ImGui::IsItemClicked())
        selected = 1;

    ImGui::NewLine();

    if (selected == 0) {
        m_off_line = true;
        const bool openAvailable = !m_connected_devices.empty();
        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
            static std::string fileName;
            ImGuiExtensions::ButtonColorChanger colorChanger(
                ImGuiExtensions::ButtonColor::Green, openAvailable);
            if (ImGuiExtensions::ADIButton("Open")) {
                
                int FilterIndex = 0;
                std::string fs = openADIFileName(
                    "ADI ToF Config Files\0*.adcam\0", nullptr, FilterIndex);
                LOG(INFO) << "Load File selected: " << fs;

                if (!fs.empty()) {

                    RefreshDevices();

                    m_is_open_device = false;
                    m_selected_device_index = 0;
                    fileName = fs;
                    m_off_line_frame_index = 0;
                    initCameraWorker =
                        std::thread([this, fs]() { InitCamera(fs); });
                }
            }
            ImGui::SameLine();
            if (ImGuiExtensions::ADIButton("Play", !m_is_open_device)) {
                m_off_line_frame_index = 0;
                m_frame_window_position_state = 0;
                m_view_selection_changed = m_view_selection;
                m_is_playing = true;
                ini_params.clear();
            }
            ImGui::NewLine();
            ImGui::Text("File selected: %s", fileName.c_str());
        }

    } else {
        m_off_line = false;
        ImGuiExtensions::ADIComboBox(
            "Camera", "(No available devices)", ImGuiComboFlags_None,
            m_connected_devices, &m_selected_device_index, m_is_open_device);
        //If a device is found, then set the first one found
        if (!m_connected_devices.empty() && m_selected_device_index == -1) {
            m_selected_device_index = 0;
            m_is_open_device = true;
        }

        ImGui::NewLine();
        bool _noConnected = m_connected_devices.empty();
        if (ImGuiExtensions::ADIButton("Refresh", _noConnected)) {
            m_is_open_device = false;
            m_cameraWorkerDone = false;
            RefreshDevices();
        }

        ImGui::SameLine();

        const bool openAvailable = !m_connected_devices.empty();
        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
            ImGuiExtensions::ButtonColorChanger colorChanger(
                ImGuiExtensions::ButtonColor::Green, openAvailable);
            if (ImGuiExtensions::ADIButton("Open", m_is_open_device) &&
                0 <= m_selected_device_index) {

                m_is_open_device = false;
                std::string fs;
                initCameraWorker =
                    std::thread([this, fs]() { InitCamera(fs); });
            }
        }

        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Close", !m_is_open_device)) {
            StopPlayback();
            CameraStop();
            m_cameraWorkerDone = false;
            m_callback_initialized = false;
            m_cameraModes.clear();
            _cameraModes.clear();
            if (initCameraWorker.joinable()) {
                initCameraWorker.join();
            }
            m_view_instance.reset();
            RefreshDevices();
        }
        ImGui::NewLine();

        if (ImGuiExtensions::ADICheckbox("Max FPS Network Test (Debug)",
                                         &m_network_link_test,
                                         m_is_open_device)) {
            if (m_network_link_test) {
                m_ip_suffix = ":netlinktest";
            } else {
                m_ip_suffix.clear();
            }
            RefreshDevices();
        }
        ImGui::NewLine();

        if (m_cameraWorkerDone) {
            m_is_open_device = false;
            if (!m_is_playing) {
                ImGui::NewLine();
                DrawBarLabel("Mode Selection");
                ImGui::NewLine();
                ImGuiExtensions::ADIComboBox(
                    "", "Select Mode", ImGuiSelectableFlags_None,
                    m_cameraModesDropDown, &m_mode_selection, true);

                ImGuiExtensions::ButtonColorChanger colorChangerPlay(
                    m_custom_color_play, !m_is_playing);
                ImGui::NewLine();
                if (ImGuiExtensions::ADIButton("Start", !m_is_playing)) {
                    m_frame_window_position_state = 0;
                    m_view_selection_changed = m_view_selection;
                    m_is_playing = true;
                    ini_params.clear();
                }
            }
        }
    }
    ImGui::End();
}