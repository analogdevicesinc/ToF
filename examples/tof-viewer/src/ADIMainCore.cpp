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

#include "roboto-bold.h"
#include "roboto-regular.h"
#include "roboto-semicondensed-bold.h"

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

extern ImFont* g_font_regular;
extern ImFont* g_font_bold;
extern ImFont* g_font_bold_large;

ADIMainWindow::ADIMainWindow() : m_skip_network_cameras(true) {
    /********************/
    struct stat info;
    std::string folderName = "log";
    if (stat(folderName.c_str(), &info) != 0) { // If no folder named log is found
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
    std::string wholeLogPath;
    char timebuff[100];
    time_t timeNow = time(0);
    struct tm *timeinfo;
    time(&timeNow);
    timeinfo = localtime(&timeNow);
    strftime(timebuff, sizeof(timebuff), "%Y%m%d_%H%M%S", timeinfo);
    // Concatenate the whole path
    wholeLogPath = folderName;
#ifdef _WIN32
    wholeLogPath += "\\"; // Ensure the path ends with a slash
#elif __linux__
    wholeLogPath += "/"; // Ensure the path ends with a slash
#endif
    wholeLogPath += "log_" + std::string(timebuff) + ".txt";

    /********************/
    m_file_stream = freopen(wholeLogPath.c_str(), "w", stderr); // Added missing pointer
    setvbuf(m_file_stream, 0, _IONBF, 0);               // No Buffering
    m_file_input = fopen(wholeLogPath.c_str(), "r");

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
    ImPlot::DestroyContext();
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

bool ADIMainWindow::StartImGUI(const ADIViewerArgs& args) {
    // Setup window
    glfwSetErrorCallback(glfw_error_callback); //Error Management
    if (!glfwInit()) {
        return false;
    }

    // Decide GL+GLSL versions
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
    glfwWindowHint(GLFW_DEPTH_BITS, 24);

    std::string version = aditof::getApiVersion();
    std::string _title = "Analog Devices, Inc. Time of Flight Main Window v" +
        version; //Default name

    // Create window with graphics context
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

    m_dict_win_position["main"].width = 1580.0f;
    m_dict_win_position["main"].height = 1080.0f;

    window = glfwCreateWindow(m_dict_win_position["main"].width, m_dict_win_position["main"].height, _title.c_str(), NULL, NULL);

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
    ImPlot::CreateContext(); // ← REQUIRED for ImPlot
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;			// Enable Docking
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    // Keyboard Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; //
    // Enable Gamepad Controls

    if (args.HighDpi) {
        m_dpi_scale_factor = HIGHDPISCALAR;
    } else {
        m_dpi_scale_factor = NORMALDPISCALAR;
    }
    SetDpi();

    m_dict_win_position["info"].x = 5.0f;
    m_dict_win_position["info"].y = 25.0f;
    m_dict_win_position["info"].width = 300.0f;
    m_dict_win_position["info"].height = 800.0f;

    m_dict_win_position["control"].x = m_dict_win_position["info"].width + 10;
    m_dict_win_position["control"].y = m_dict_win_position["info"].y;
    m_dict_win_position["control"].width = m_dict_win_position["info"].width;
    m_dict_win_position["control"].height = m_dict_win_position["info"].height;

    m_dict_win_position["fr-main"].x =
        WindowCalcX(m_dict_win_position["control"], 10.0f);
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

    m_dict_win_position["plotA"].x = m_dict_win_position["fr-main"].x;
    m_dict_win_position["plotA"].y = WindowCalcY(m_dict_win_position["fr-main"], 10.0f);
    m_dict_win_position["plotA"].width = m_dict_win_position["fr-main"].width;
    m_dict_win_position["plotA"].height = 315.0f;

    m_xyz_position = &m_dict_win_position["fr-main"];
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
	//glDeleteTextures(1, &m_gl_pc_colourTex); // TODO: Find out why deleting this causes issues.
    //glDeleteTextures(1, &m_gl_pc_depthTex);  // TODO: Find out why deleting this causes issues.
    glDeleteVertexArrays(1, &m_view_instance->vertexArrayObject);
    glDeleteBuffers(1, &m_view_instance->vertexBufferObject);
    glDeleteProgram(m_view_instance->pcShader.Id());
    m_view_instance->pcShader.RemoveShaders();
}

ImFont * ADIMainWindow::LoadFont(const unsigned char *ext_font, const unsigned int ext_font_len, const float size) {
    ImFont *font;
    bool isFontLoaded = false;
    unsigned char* buffer = new(std::nothrow) unsigned char[ext_font_len];
    if (buffer == nullptr) {
        LOG(ERROR) << "Failed to allocate memory for Roboto Regular font!";
    }
    else {
        std::memcpy(buffer, ext_font, ext_font_len);
        font = ImGui::GetIO().Fonts->AddFontFromMemoryTTF(buffer, ext_font_len, size * m_dpi_scale_factor);
        if (!font) {
            LOG(ERROR) << "Failed to load font!";
            delete[] buffer; // Clean up memory if font loading fails
        }
        isFontLoaded = true;
    }
    if (!isFontLoaded) {
        font = ImGui::GetIO().FontDefault;
    }

    return font;
}

void ADIMainWindow::SetDpi() {
    ImGui::GetStyle().ScaleAllSizes(m_dpi_scale_factor);

    // ImGui doesn't automatically scale fonts, so we have to do that ourselves
    //
    ImFontConfig fontConfig;
    constexpr float defaultFontSize = 13.0f;
    fontConfig.SizePixels = defaultFontSize * m_dpi_scale_factor;
    ImGui::GetIO().Fonts->AddFontDefault(&fontConfig);

    g_font_regular = LoadFont(Roboto_Regular_ttf, Roboto_Regular_ttf_len, 12.0f);
    g_font_bold = LoadFont(Roboto_Bold_ttf, Roboto_Bold_ttf_len, 12.0f);
    g_font_bold_large = LoadFont(Roboto_Bold_ttf, Roboto_Bold_ttf_len, 18.0f);

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
        DisplayHelp();
        if (m_is_playing) {
            CameraPlay(m_mode_selection, m_view_selection);
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

        if (getIsWorking()) {
			Spinner("Working...", 10.0f, 2.0f, IM_COL32(255, 255, 255, 255));
        }

        /***************************************************/
        // Rendering
        static bool flashWindow = false;
        static float flashTimer = 0.0f;
        static const float flashDuration = 0.2f; // seconds
        ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.00f, 1.00f);

        if (m_flash_main_window) {
            flashWindow = true;
            flashTimer = flashDuration;
			m_flash_main_window = false;
        }

        float deltaTime = ImGui::GetIO().DeltaTime;
        if (flashWindow) {
            flashTimer -= deltaTime;
            if (flashTimer <= 0.0f) {
                flashWindow = false;
            }
        }

        if (flashWindow) {
            clear_color = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
        }
        else {
            clear_color = ImVec4(0.0f, 0.0f, 0.00f, 1.00f);
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}

void ADIMainWindow::ShowMainMenu() {
    static bool show_app_log = false;
    static bool show_help_window = false;

    if (show_app_log) {
        ShowLogWindow(&show_app_log);
    }

    if (show_help_window) {
        ImGui::OpenPopup("Help Window");
        show_help_window = false;
    }

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("?")) {
            if (ImGui::MenuItem("Help")) {
                show_help_window = true;
            }
            ImGui::MenuItem("Debug Log", nullptr, &show_app_log);
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(window, true);
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

            m_ini_params.clear();
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

        }
    }
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

    static float wizard_height = 360.0f;
    
    centreWindow(450.0f * m_dpi_scale_factor, wizard_height * m_dpi_scale_factor);

    ImGui::Begin("Camera Selection Wizard", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);

    static uint32_t selected = 1;
    uint32_t state_change_check = selected;

    ImGui::RadioButton("Saved Stream", selected == 0);
    if (ImGui::IsItemClicked())
        selected = 0;
    ImGui::SameLine();
    ImGui::RadioButton("Live Camera", selected == 1);
    if (ImGui::IsItemClicked())
        selected = 1;

    if (selected == 1 && selected != state_change_check) {
        if (m_is_open_device) {
        }
    }

    ImGui::NewLine();

    if (selected == 0) {
#pragma region WizardOffline
        if (wizard_height < 300)
            wizard_height += 20;
        else if (wizard_height > 300)
            wizard_height -= 20;

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

                    //m_is_open_device = true;
                    m_is_playing = false;
                    m_is_open_device = false;
                    m_selected_device_index = 0;
                    fileName = fs;
                    m_off_line_frame_index = 0;
                    initCameraWorker =
                        std::thread([this, fs]() { InitCamera(fs); });
                }
            }
            ImGui::SameLine();
            if (ImGuiExtensions::ADIButton("Start Streaming", m_is_open_device)) {

                // Deallocate frame memory such that it can be reallocated for the 
                //  correct frame size in case there was a change in mode.
                m_view_instance->cleanUp();

                // PRB25 
                auto camera = GetActiveCamera(); //already initialized on constructor
                if (camera != nullptr) {

                    m_offline_change_frame = true;
                    camera->startPlayback(fileName);

                    m_off_line_frame_index = 0;
                    m_frame_window_position_state = 0;
                    m_view_selection_changed = m_view_selection;
                    m_is_playing = true;
                    m_ini_params.clear();
                }
                else {
                    LOG(ERROR) << "Camera not initialized!";
                    return;
                }
            }
            if (m_is_open_device) {
                NewLine(5.0f);
                ImGui::Text("File selected");
                ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 400); // Wrap at 400px
                ImGui::TextWrapped("  %s", fileName.c_str());
                ImGui::PopTextWrapPos();
                NewLine(5.0f);
            }
        }
#pragma endregion // WizardOffline
    }
    else {
#pragma region WizardOnline
        m_off_line = false;
        ImGuiExtensions::ADIComboBox(
            "Camera", "(No available devices)", ImGuiComboFlags_None,
            m_connected_devices, &m_selected_device_index, m_is_open_device);
        //If a device is found, then set the first one found
        if (!m_connected_devices.empty() && m_selected_device_index == -1) {
            m_selected_device_index = 0;
            //m_is_open_device = true;
        }

        NewLine(5.0f);
        bool _noConnected = m_connected_devices.empty();
        if (ImGuiExtensions::ADIButton("Refresh", !m_is_open_device)) {
            m_is_open_device = false;
            m_cameraWorkerDone = false;
            RefreshDevices();
        }

        ImGui::SameLine();

        const bool openAvailable = !m_connected_devices.empty();
        { // Use block to control the moment when ImGuiExtensions::ButtonColorChanger gets destroyed
            ImGuiExtensions::ButtonColorChanger colorChanger(
                ImGuiExtensions::ButtonColor::Green, openAvailable);
            if (ImGuiExtensions::ADIButton("Open", !m_is_open_device) &&
                0 <= m_selected_device_index) {

                m_is_open_device = true;
                std::string fs;
                initCameraWorker =
                    std::thread([this, fs]() { InitCamera(fs); });
            }
        }

        ImGui::SameLine();
        if (ImGuiExtensions::ADIButton("Close", m_is_open_device)) {
            CameraStop();
            if (initCameraWorker.joinable()) {
                initCameraWorker.join();
                m_cameraModes.clear();
                _cameraModes.clear();
            }
            m_view_instance->cleanUp();
            m_view_instance.reset();
            m_callback_initialized = false;
            m_cameraWorkerDone = false;
            m_is_open_device = false;
            RefreshDevices();
        }
        NewLine(5.0f);

        if (ImGuiExtensions::ADICheckbox("Max FPS Network Test (Debug)",
            &m_network_link_test,
            m_is_open_device)) {
            if (m_network_link_test) {
                m_ip_suffix = ":netlinktest";
            }
            else {
                m_ip_suffix.clear();
            }
            RefreshDevices();
        }
        NewLine(5.0f);

        if (m_cameraWorkerDone) {
            //m_is_open_device = false;
            if (!m_is_playing) {

                NewLine(5.0f);

                DrawBarLabel("Mode Selection");

                NewLine(10.0f);

                static bool show_dynamic_mode_switch = false;
#ifdef ENABLE_DYNAMIC_MODE_SWITCHING
                ImGui::Toggle(!show_dynamic_mode_switch ? "Switch to Dynamic Mode" : "Switch to Standard Mode",
                    &show_dynamic_mode_switch);
#endif //ENABLE_DYNAMIC_MODE_SWITCHING

                if (show_dynamic_mode_switch) {
#ifdef ENABLE_DYNAMIC_MODE_SWITCHING
#pragma region WizardOnlineDynamicMode
                    if (wizard_height < 540)
                        wizard_height += 20;
                    else if (wizard_height > 540)
                        wizard_height -= 20;

                    // TODO: Add non-Crosby repeat count

                    static int32_t mode_selections[] = { 1, 1, 1, 1, 1, 1, 1, 1 };
                    static int32_t mode_repeat[] = { 1, 1, 1, 1, 1, 1, 1, 1 };

                    for (int32_t idx = 0; idx < 8; idx++) {
                        ImGui::SetNextItemWidth(180 * m_dpi_scale_factor);

                        std::string slot = "Slot " + std::to_string(idx + 1) + " mode";

                        ImGuiExtensions::ADIComboBox(
                            slot.c_str(), "Select Mode", ImGuiSelectableFlags_None,
                            m_cameraModesDropDown, &mode_selections[idx], true);

                        ImGui::SameLine();


                        std::string repeat = "Repeat " + std::to_string(idx + 1);
                        ImGui::SetNextItemWidth(60 * m_dpi_scale_factor);
                        if (ImGui::BeginCombo(repeat.c_str(), std::to_string(mode_repeat[idx]).c_str())) {
                            for (int i = 0; i <= 15; ++i) {
                                bool isSelected = (mode_repeat[idx] == i);
                                if (ImGui::Selectable(std::to_string(i).c_str(), isSelected))
                                    mode_repeat[idx] = i;

                                if (isSelected)
                                    ImGui::SetItemDefaultFocus();
                            }
                            ImGui::EndCombo();
                        }
                    }

                    ImGuiExtensions::ButtonColorChanger colorChangerPlay(
                        m_custom_color_play, !m_is_playing);

                    NewLine(5.0f);
                    if (ImGuiExtensions::ADIButton("Start Streaming", !m_is_playing)) {

                        std::vector<std::pair<uint8_t, uint8_t>> seqence;

                        seqence.push_back(std::make_pair(mode_selections[0], mode_repeat[0]));
                        seqence.push_back(std::make_pair(mode_selections[1], mode_repeat[1]));
                        seqence.push_back(std::make_pair(mode_selections[2], mode_repeat[2]));
                        seqence.push_back(std::make_pair(mode_selections[3], mode_repeat[3]));
                        seqence.push_back(std::make_pair(mode_selections[4], mode_repeat[4]));
                        seqence.push_back(std::make_pair(mode_selections[5], mode_repeat[5]));
                        seqence.push_back(std::make_pair(mode_selections[6], mode_repeat[6]));
                        seqence.push_back(std::make_pair(mode_selections[7], mode_repeat[7]));

                        // Deallocate frame memory such that it can be reallocated for the 
                        //  correct frame size in case there was a change in mode.
                        m_view_instance->cleanUp();
                        auto camera = GetActiveCamera();

                        camera->adsd3500setEnableDynamicModeSwitching(true);
                        camera->adsds3500setDynamicModeSwitchingSequence(seqence);

                        m_frame_window_position_state = 0;
                        m_view_selection_changed = m_view_selection;
                        m_is_playing = true;
                        m_ini_params.clear();
                    }
#pragma endregion // WizardOnlineDynamicMode
#endif //ENABLE_DYNAMIC_MODE_SWITCHING
                } else {
#pragma region WizardOnlineStandardMode
                    if (wizard_height < 640)
                        wizard_height += 20;
					else if (wizard_height > 640)
						wizard_height -= 20;

                    if (ImGuiExtensions::ADIComboBox(
                        "", "Select Mode", ImGuiSelectableFlags_None,
                        m_cameraModesDropDown, &m_mode_selection, true)) {
						m_ini_params.clear();
                    }

                    NewLine(5.0f);

                    DrawBarLabel("Configuration");

                    NewLine(5.0f);

                    if (ImGuiExtensions::ADIButton("Load Config", !m_is_playing)) {

                        ShowLoadAdsdParamsMenu();
                    }

					ImGui::SameLine();

                    if (ImGuiExtensions::ADIButton("Reset Parameters", m_is_open_device)) {
                        auto camera = GetActiveCamera();
                        if (camera) {
                            camera->resetDepthProcessParams();
                            m_ini_params.clear();
                        }
                    }

                    NewLine(5.0f);

                    ShowIniWindow(false);

                    NewLine(15.0f);

                    //Change colour to green
                    ImGuiExtensions::ButtonColorChanger colorChangerPlay(m_custom_color_play, !m_is_playing);

                    ImGui::Toggle(!m_enable_preview ? "Preview Off" : "Preview On", &m_enable_preview);

                    if (ImGuiExtensions::ADIButton("Start Streaming", !m_is_playing)) {

                        // Deallocate frame memory such that it can be reallocated for the 
                        //  correct frame size in case there was a change in mode.
                        if (m_view_instance) {
                            m_view_instance->cleanUp();
                        }

                        auto camera = GetActiveCamera();
                        if (camera) {
                            camera->adsd3500setEnableDynamicModeSwitching(false);
                        }

                        m_frame_window_position_state = 0;
                        m_view_selection_changed = m_view_selection;
                        m_last_mode = m_mode_selection;   // Force use of ini parameters
                        m_use_modified_ini_params = true; // Force use of ini parameters
                        m_is_playing = true;
                        m_ini_params.clear();
                    }

                }
#pragma endregion // WizardOnlineStandardMode
            }
        }
#pragma endregion // WizardOnline
    }
    ImGui::End();
}

void ADIMainWindow::centreWindow(float width, float height) {

    ImGuiIO& io = ImGui::GetIO(); // Get the display size
    ImVec2 center = ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
    ImVec2 window_size = ImVec2(width, height); // Your window size

    // Offset to truly center it
    ImVec2 window_pos = ImVec2(center.x - window_size.x * 0.5f,
        center.y - window_size.y * 0.5f);

    // Set position and size before Begin()
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(window_size, ImGuiCond_Always);
}

// Minimal spinner function for ImGui (circle dots)
void ADIMainWindow::Spinner(const char* label, float radius, int thickness, ImU32 color) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
        return;
    ImGuiContext& g = *ImGui::GetCurrentContext();
    const ImGuiID id = window->GetID(label);

    ImVec2 pos = ImGui::GetCursorScreenPos();
    float t = (float)g.Time;
    int num_segments = 30;
    float angle_min = IM_PI * 2.0f * (t * 0.8f);
    float angle_max = IM_PI * 2.0f * ((t * 0.8f) + 1.0f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->PathClear();
    for (int i = 0; i < num_segments; i++) {
        float a = angle_min + ((float)i / (float)num_segments) * (angle_max - angle_min);
        draw_list->PathLineTo(ImVec2(
            pos.x + radius + cosf(a) * radius,
            pos.y + radius + sinf(a) * radius));
    }
    draw_list->PathStroke(color, 0, thickness);
    ImGui::Dummy(ImVec2((radius + thickness) * 2, (radius + thickness) * 2));
}
