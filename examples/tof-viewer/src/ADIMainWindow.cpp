﻿/********************************************************************************/
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
#include <glog/logging.h>
#include <stdio.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <cmath>

#include <ADIFSFOptions.h>
#include <aditof/system.h>
#include <cJSON/cJSON.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#include <io.h>
#include "psapi.h"
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
#include <GLFW/glfw3.h>
#include <CompanyLogo.png.h>
#include <CompanyIcon.png.h>

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

extern std::vector<std::string> customFilters;

static int numProcessors;
GLFWimage icons[1];
GLFWimage logos[1];
GLuint logo_texture;
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
        if (mkdir("log",0777)) { // Create local folder where all logs will be filed
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

	// Parse config file for this application
    //Parse config.json
    std::ifstream ifs("tof-viewer_config.json");
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
    if (view != nullptr && (view->m_ctrl->m_recorder->isFSFRecording ||
                            !view->m_ctrl->m_recorder->m_finishRecording)) {
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
	glfwSetErrorCallback(glfw_error_callback);//Error Management
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
	std::string _title = "Analog Devices, Inc. Time of Flight Main Window v" + version;//Default name
	
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
    icons[0].pixels = stbi_load_from_memory(company_icon_buffer, icon_size, &icons[0].width, &icons[0].height,
            &components, 0);
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
	
	RefreshDevices();
	
	//Look for Company Logo
	// Create a OpenGL texture identifier	
	logo_texture = -1;
	logos[0].pixels = stbi_load_from_memory(company_logo_buffer, logo_size, &logos[0].width, &logos[0].height, &components, 0);
	glGenTextures(1, &logo_texture);
	glBindTexture(GL_TEXTURE_2D, logo_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Upload pixels into texture
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, logos[0].width, 
				logos[0].height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
				logos[0].pixels);
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
	static bool show_app_log = true;
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
		/***************************************************/
		//Create windows here:
		showMainMenu();
		showOpenDeviceWindow();
		if (isPlaying) {
			PlayCCD(modeSelection, viewSelection);
			if (view->m_ctrl->panicStop) {
				stopPlayCCD();
			}
		} else if (isPlayRecorded) {
			PlayRecorded();
		}
		showLogWindow(&show_app_log);	
		if (_fsfShowPbOpWin) {
			ShowFSFOptionsGUI(mainWindowWidth, mainWindowHeight, 
							  _fsfShowPbOpWin, _startFSFPb, 
							  view->m_ctrl->m_recorder->_streamEnable);			
		}
		if (_fsfShowRecordOpWin) {
			//SetFSFStreamInfo(mainWindowWidth, mainWindowHeight, _fsfShowRecordOpWin, _startFSFRec, view->m_ctrl->m_recorder->_streamEnable);
			_fsfShowRecordOpWin = false;
			_startFSFRec = true;
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
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("Options")) {
			if (ImGui::MenuItem("Reset") && view != nullptr) {
				stopPlayback();
				stopPlayCCD();
				isADICCD = false;
				isADIToF = false;
				cameraWorkerDone = false;
				m_cameraModes.clear();
				_cameraModes.clear();
				if (initCameraWorker.joinable()) {
					initCameraWorker.join();
				}
				RefreshDevices();
			}
			ImGui::Separator();

			if (ImGui::MenuItem("Quit")) {
				glfwSetWindowShouldClose(window, true);
			}
			
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}
}

void ADIMainWindow::RefreshDevices() {
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
        m_system.getCameraListAtIp(m_camerasList, m_cameraIp);
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

void ADIMainWindow::showOpenDeviceWindow() {
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);	
	setWindowPosition(0.0, offsetfromtop);
	setWindowSize(300.0, 590);
	ImGui::Begin("Device", NULL, 
				  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
	
	if (ImGui::TreeNode("Open Device")) {
		ImGuiExtensions::ADIComboBox("Device", "(No available devices)",
			ImGuiComboFlags_None, m_connectedDevices,
			&m_selectedDevice, _isOpenDevice);
		
		//If a device is found, then set the first one found
		if (!m_connectedDevices.empty() && m_selectedDevice == -1) {
			m_selectedDevice = 0;
			_isOpenDevice = true;
		}

		if (ImGui::Button("Refresh Devices")) {
			RefreshDevices();
		}

		ImGui::SameLine();

		const bool openAvailable = !m_connectedDevices.empty();
		{
			
			ImGuiExtensions::ButtonColorChanger colorChanger(
				ImGuiExtensions::ButtonColor::Green, openAvailable);
			if (ImGuiExtensions::ADIButton("Open Device", /*openAvailable*/_isOpenDevice && 
											m_configFiles.size() > 0) && 0 <= m_selectedDevice) {
				if (isPlayRecorded) {
					stopPlayback();
				}

				if (!m_connectedDevices[m_selectedDevice].second.find("ADI CCD")) {
					//Init CCD Device here
					_isOpenDevice = false;
					InitCCDCamera();
					isADICCD = true;
					isADIToF = false;
				}
				else if (!m_connectedDevices[m_selectedDevice].second.find("ToF Camera")) {
					//Init CMOS Device here
					_isOpenDevice = false;
					initCameraWorker =
						std::thread(std::bind(&ADIMainWindow::InitCamera, this));
					isADICCD = false;
					isADIToF = true;
				}
                else if (!m_connectedDevices[m_selectedDevice].second.find("Fake Device")) {
                    isADICCD = false;
                    isADIToF = false;
				}
			}
		}

		if (!m_connectedDevices.empty()) {
			const float width = ImGui::GetWindowWidth();
			const float combo_width = width * 0.5f;
			ImGui::SetNextItemWidth(combo_width);
			ImGuiExtensions::ADIComboBox("Config Files", "No Config Files",
				ImGuiSelectableFlags_None, m_configFiles,
				&configSelection, _isOpenDevice);
		}
		//ImGui::SameLine();
#if defined(Debug) && defined(_WIN32)
		cpuUsage = getCurrentValue();
		std::string cpu = std::to_string(cpuUsage);
		ImGui::Text("CPU Usage: %s %%",cpu.c_str());

		/*Total Virtual Memory*/
		MEMORYSTATUSEX memInfo;
		memInfo.dwLength = sizeof(MEMORYSTATUSEX);
		GlobalMemoryStatusEx(&memInfo);
		DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile;

		/*Total Virtual Memory currently used*/
		DWORDLONG virtualMemUsed = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;

		/*Virtual memory by current process*/
		PROCESS_MEMORY_COUNTERS_EX pmc;
		GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
		SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

		/*Physical Memory currently used by current process:*/
		SIZE_T physMemUsedByMe = pmc.WorkingSetSize;

		/*Physical Memory currently used*/
		DWORDLONG physMemUsed = memInfo.ullTotalPhys - memInfo.ullAvailPhys;

		SIZE_T totalMemUsedByProcess = virtualMemUsedByMe - physMemUsedByMe;
		//TODO: Find an accurate way of displaying the memory
		ImGui::Text("Memory Used by Process: %s MB", std::to_string((double)physMemUsedByMe /1000000).c_str());
#endif
		if (isADICCD) {
			ImGui::NewLine();
			ImGui::Separator();
			ImGui::NewLine();
			ImGui::SetNextTreeNodeOpen(cameraOptionsTreeEnabled, 
									   ImGuiCond_FirstUseEver);
			if (ImGui::TreeNode("ADI CCD Camera Options")) {
				ImGui::Text("Mode:");
				ImGuiExtensions::ADIRadioButton("Near", &modeSelection, 0);
				ImGuiExtensions::ADIRadioButton("Medium", &modeSelection, 1);
				ImGuiExtensions::ADIRadioButton("Far", &modeSelection, 2);

				ImGui::NewLine();
				ImGui::Text("View Depth and Active Brightness:");
				ImGuiExtensions::ADIRadioButton("Show in separated windows", &viewSelection, 0);
				ImGuiExtensions::ADIRadioButton("Blend both in one window", &viewSelection, 1);
				
				ImGui::NewLine();
				ImGui::Text("Video:");
				float customColorPlay = 0.4f;
				float customColorStop = 0.0f;
				ImGuiExtensions::ButtonColorChanger colorChangerPlay(customColorPlay, !isPlaying);
				if (ImGuiExtensions::ADIButton("Play", !isPlaying)) {
					viewSelectionChanged = viewSelection;
					isPlaying = true;
				}
				ImGui::SameLine();
				ImGuiExtensions::ButtonColorChanger colorChangerStop(customColorStop, isPlaying);
				if (ImGuiExtensions::ADIButton("Stop", isPlaying)) {
					isPlaying = false;
					stopPlayCCD();
				}

				ImGui::TreePop();
			}
		}		
		if (isADIToF && cameraWorkerDone) {
			ImGui::NewLine();
			ImGui::Separator();
			ImGui::NewLine();
			ImGui::SetNextTreeNodeOpen(cameraOptionsTreeEnabled, 
					/*ImGuiCond_FirstUseEver*/ImGuiCond_Always);
			
			_isOpenDevice = false;
			if (ImGui::TreeNode("ToF Camera Options")) {
				ImGui::Text("Mode:");
				const float width = ImGui::GetWindowWidth();
				const float combo_width = width * 0.35f;
				ImGui::SetNextItemWidth(combo_width);
				ImGuiExtensions::ADIComboBox("Mode Options",
					"Select Mode", ImGuiSelectableFlags_None,
					m_cameraModes, &modeSelection, true);

				ImGui::NewLine();
				ImGui::Text("View Options:");
				ImGuiExtensions::ADIRadioButton("Active Brightness and Depth", &viewSelection, 0);
				ImGui::NewLine();
				ImGuiExtensions::ADIRadioButton("Point Cloud and Depth", &viewSelection, 1);
				ImGui::NewLine();
				ImGui::Text("Video:");
								
				ImGuiExtensions::ButtonColorChanger colorChangerPlay(customColorPlay, !isPlaying);
				if (ImGuiExtensions::ADIButton("Play", !isPlaying && !isPlayRecorded)) {
					viewSelectionChanged = viewSelection;
					isPlaying = true;
				}
				ImGui::SameLine();
				ImGuiExtensions::ButtonColorChanger colorChangerStop(customColorStop, isPlaying);
				if (ImGuiExtensions::ADIButton("Stop", isPlaying)) {
					isPlaying = false;
					isPlayRecorded = false;
					stopPlayCCD();
					if (isRecording) {
						view->m_ctrl->stopRecording();
						isRecording = false;
					}
				}

				ImGui::TreePop();
				ShowRecordTree();
			}
		}	

		ImGui::TreePop();
	}

	ShowPlaybackTree();
	//Logo Window
	setWindowPosition(0.0, 628);
	setWindowSize(300.0, 90.0);
	ImGui::Begin("Company Logo", NULL, 
				  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | 
				  ImGuiWindowFlags_NoScrollbar);
	
	float scale = 1.0f;
	if (logos[0].pixels != nullptr) {
		if (logos[0].width > 480.0f) {
			if (_isHighDPI) {
				scale = 480.0f / logos[0].width;
			}
			else {
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
		
		ImGui::Image((void*)(intptr_t)logo_texture, 
					ImVec2(logos[0].width * scale, logos[0].height * scale));
	}
	
	ImGui::End(); //Logo END

	ImGui::End();
}

void ADIMainWindow::ShowPlaybackTree() {
	/**********/
	//Playback
	ImGui::NewLine();
	ImGui::Separator();
	ImGui::NewLine();
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);
	
	if (ImGui::TreeNode("Playback Options")) {
		float customColorOpenRec = 0.22f;
		ImGuiExtensions::ButtonColorChanger colorChangerPlay(
			customColorOpenRec, !isPlayRecorded && !isPlaying);

		if (ImGuiExtensions::ADIButton("Open Recording", !isPlayRecorded && !isPlaying && !_fsfShowPbOpWin)) {
			customFilter = std::string("FSF Files (*.fsf)\0*.fsf*\0 Raw Files (*.raw)\0*.raw*\0 All Files (*.*)\0*.*\0", 72);
            std::string filters[] = { "fsf", "raw" };    
            customFilters.clear();
            std::copy(std::begin(filters), std::end(filters), std::back_inserter(customFilters));
			std::string path = openADIFileName().c_str();
			if (!path.empty()) {
                if (view == NULL) {
					view = std::make_shared<adiviewer::ADIView>(
                    m_controller, "Record Viewer");
                }
				if (path.find(".fsf") != std::string::npos) { //If fsf File was found
					_fsfShowPbOpWin = true;
					_getFSFInfo = true;
					fsfPath = path;
				} else {
					view->m_ctrl->startPlayback(path, recordingSeconds);
					initOpenGLIRTexture();
					initOpenGLDepthTexture();
					isPlayRecorded = true;
					_usingFSF = false;
				}
				viewSelection = 0; //default view to depth/ab
				_isOpenDevice = false;
				cameraOptionsTreeEnabled = false;
			} else {
				_isOpenDevice = true;
				cameraOptionsTreeEnabled = true;
			}
		}
		if (_getFSFInfo) {
			FSFStreamEnable checkAvailability;
			view->m_ctrl->m_recorder->_streamEnable = 
				checkAvailability;//init this variable again.
			GetFSFInfo(fsfPath.c_str(), checkAvailability);
			_getFSFInfo = false;
		}
		if (_startFSFPb) {
			view->m_ctrl->startPlayback(fsfPath, recordingSeconds);
			initOpenGLIRTexture();
			initOpenGLDepthTexture();
			initOpenGLPointCloudTexture();
			isPlayRecorded = true;
			_startFSFPb = false;
			_usingFSF = true;
		}
		
		std::string playbackButtonText = isPlayRecordPaused ? "Paused" : "Pause";
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

		if (ImGuiExtensions::ADIButton(playbackButtonText.c_str(), isPlayRecorded)) {
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
			stopPlayback();
			_usingFSF = false;
		}
		if (isPlayRecorded) {
			ImGui::NewLine();
			if (_usingFSF) {
				ImGuiExtensions::ADISliderInt("Progress", &view->m_ctrl->m_recorder->currentPBPos, 0, view->m_ctrl->m_recorder->fileHeader.nFrames - 1, "%d", true);
			} else { //Using RAW					
				rawSeeker = (view->m_ctrl->m_recorder->currentPBPos) / (((int)view->m_ctrl->m_recorder->m_frameDetails.height) * ((int)view->m_ctrl->m_recorder->m_frameDetails.width) * sizeof(uint16_t) * 2);
				ImGuiExtensions::ADISliderInt("Progress", &rawSeeker, 0, (view->m_ctrl->m_recorder->m_numberOfFrames / 2) - 1, "%d", true);
				view->m_ctrl->m_recorder->currentPBPos = rawSeeker * (((int)view->m_ctrl->m_recorder->m_frameDetails.height) * ((int)view->m_ctrl->m_recorder->m_frameDetails.width) * sizeof(uint16_t) * 2);				
			}

			ImGui::Text("View Options:");
			ImGuiExtensions::ADIRadioButton("Active Brightness and Depth", &viewSelection, 0, displayIR || displayDepth);
			ImGui::NewLine();
			ImGuiExtensions::ADIRadioButton("Point Cloud and Depth", &viewSelection, 1, pointCloudEnable);
			ImGui::NewLine();
		}

		if (0) {
			//USE ONLY IN DEBUG MODE: Highly Experimental. May use in the future
			ImGui::NewLine();
			if (ImGuiExtensions::ADIButton("Open *.bin Recording", !isPlayRecorded && !isPlaying && !_fsfShowPbOpWin)) {
				customFilter = std::string("Raw Files (*.bin)\0*.bin*\0 All Files (*.*)\0*.*\0", 46);
				std::string path = openADIFileName().c_str();
				if (!path.empty()) {
					if (path.find(".bin") != std::string::npos) { //If bin File was found
						int frames = 1;
						int width = 1024;
						int height = 1024;
						view->m_ctrl->startPointCloudBinToFSFConversion(
							path, frames, width, height);
					}
				}
			}
		}

		ImGui::TreePop(); //Playback
	}
}

void ADIMainWindow::stopPlayback() {
	isPlaying = false;
	isPlayRecorded = false;
	displayDepth = true;
	displayIR = true;
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

void ADIMainWindow::ShowRecordTree() {
	/**********/
	//Recording
	ImGui::NewLine();
	ImGui::Separator();
	ImGui::NewLine();
	ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);

	if (ImGui::TreeNode("Record Options")) {
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
		
		if (ImGuiExtensions::ADIButton("Start Recording", !isRecording && !isPlayRecorded)) {
            fs::path NPath = fs::current_path();

			customFilter = std::string("FSF Files (*.fsf)\0*.fsf*\0 Raw Files (*.raw)\0*.raw*\0 All Files (*.*)\0*.*\0", 72);
            std::string filters[] = { "fsf", "raw" };    
            customFilters.clear();
            std::copy(std::begin(filters), std::end(filters), std::back_inserter(customFilters));
            
			std::string tempPath = NPath.string();
    		char time_buffer[128];
			struct tm timeinfo;
			time_t rawtime;
			time (&rawtime);
#ifdef _WIN32
			localtime_s(&timeinfo, &rawtime);
#else
			localtime_r(&rawtime, &timeinfo);
#endif
    		strftime(time_buffer,sizeof(time_buffer),"%Y%m%d%H%M", &timeinfo);   			
			tempPath += "\\frames" + std::string(time_buffer);

			int filterIndex = 0;
            char tempbuff[MAX_PATH];
			tempPath.copy(tempbuff, tempPath.length(), 0);
			tempbuff[tempPath.length()] = '\0';
			std::string saveFile = getADIFileName(NULL, tempbuff, filterIndex);
			if (!saveFile.empty()) {
				//Add a file extension
				switch (filterIndex) {
				case 1: //FSF file
					saveFile += ".fsf";
					fsfPath = saveFile; 
					break;
				case 2: //Raw file
					saveFile += ".raw";
					if (!isPlaying) {
						//"Press" the play button, in case it is not pressed.
						PlayCCD(modeSelection, viewSelection);//Which ever is currently selected
						isPlaying = true;
					}
					view->m_ctrl->startRecording(saveFile, view->frameHeight, 
							view->frameWidth, recordingSeconds);
					isRecording = true;
					break;
				default://custom extension					
					break;
				}				
				
				if (filterIndex == 1) { //FSF file 
					_fsfShowRecordOpWin = true;
					_setFSFInfo = true;
				} else {
					//Decide what to do here:
				}
				// Save CFG and CCB next to the recording
				auto camera = getActiveCamera();
				if (camera) {
					aditof::CameraDetails camDetails;
					camera->getDetails(camDetails);

					std::ofstream ccbFile;
					std::string ccbFilePath(camDetails.cameraId + ".ccb");
					std::ofstream cfgFile;
					std::string cfgFilePath(camDetails.cameraId + ".cfg");


					aditof::Status status = camera->setControl("saveModuleCCB", ccbFilePath);
					if (status != aditof::Status::OK) {
						LOG(INFO) << "Failed to store CCB to " << ccbFilePath;
					}

					status = camera->setControl("saveModuleCFG", cfgFilePath);
					if (status != aditof::Status::OK) {
						LOG(INFO) << "Failed to store CFG to " << cfgFilePath;
					}
				} else {
					LOG(ERROR) << "No camera found. Can't save CCB & CFG files.";
				}
			}
		}
		
		if (_setFSFInfo) {
			FSFStreamEnable setStreams;
			view->m_ctrl->m_recorder->_streamEnable = setStreams;//init this variable again.
			_setFSFInfo = false;
		}
		if (_startFSFRec) {
			if (!isPlaying) {
				//"Press" the play button, in case it is not pressed.
				PlayCCD(modeSelection, 
					viewSelection);//Which ever is currently selected
				isPlaying = true;
			}
			view->m_ctrl->startRecording(fsfPath, view->frameHeight, 
									view->frameWidth, recordingSeconds);			
			isRecording = true;
			_startFSFRec = false;
		}

		ImGui::SameLine();
		ImGuiExtensions::ButtonColorChanger colorChangerRecStop(customColorStop, isRecording);
		if (ImGuiExtensions::ADIButton("Stop Recording", isRecording)) {
			view->m_ctrl->stopRecording();
			isRecording = false;
			isPlayRecorded = false;

			stopPlayCCD();//TODO: Create a Stop ToF camera
			isPlaying = false;
		}
		ImGui::TreePop();//Record
	}
	
	if (view != NULL && view->m_ctrl->recordingFinished() && isRecording) {
		view->m_ctrl->stopRecording();
		isRecording = false;
	}
}

std::string ADIMainWindow::getCurrentPath() {
	fs::path currPath = fs::current_path();
	return currPath.string();
}

void ADIMainWindow::PlayRecorded() {
	ImGuiWindowFlags overlayFlags = /*ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |*/
		ImGuiWindowFlags_NoResize | /*ImGuiWindowFlags_AlwaysAutoResize |*/
		ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
		ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar | 
		ImGuiWindowFlags_NoBringToFrontOnFocus;

	const bool imageIsHovered = ImGui::IsItemHovered();

	if (_usingFSF) {
		displayIR = view->m_ctrl->m_recorder->_streamEnable.active_br;
		displayDepth = view->m_ctrl->m_recorder->_streamEnable.depth;
		displayPointCloud = view->m_ctrl->m_recorder->_streamEnable.x && 
							view->m_ctrl->m_recorder->_streamEnable.y && 
							view->m_ctrl->m_recorder->_streamEnable.depth;
		pointCloudEnable = displayPointCloud;
		if (viewSelection != 1) { //Display Point Cloud == 1
			displayPointCloud = false;//Don't show it
			//Display Either Active_BR or Depth OR both
			synchronizeDepthIRVideo();
		} else { //We're maybe displaying only Point Cloud
			displayIR = false;
			displayDepth = true;
			synchronizePointCloudVideo();
			synchronizeDepthIRVideo();
		}
	} else {
		displayIR = true;
		displayDepth = true;
		displayPointCloud = false;
		pointCloudEnable = false;
		synchronizeDepthIRVideo();
	}
	if (displayIR) {
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
	setIRWinPositionOnce = true;
	setDepthWinPositionOnce = true;
	setPointCloudPositionOnce = true;
	dephtWinCtr = 0;
	if (view) {
		if (view->m_ctrl) {
			view->m_ctrl->StopCapture();
			view->m_ctrl->panicStop = false;
		}
		view->m_capturedFrame = nullptr;
		view->ir_video_data = nullptr;
		view->depth_video_data = nullptr;
		view->pointCloud_video_data = nullptr;
	}
	openGLCleanUp();	
	isPlaying = false;
	isPlayRecorded = false;
}

void ADIMainWindow::openGLCleanUp() {
	glDeleteTextures(1, &ir_video_texture);
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

void ADIMainWindow::InitCamera() {
	if (view != NULL) { //Reset current imager
		LOG(INFO) << "Imager is reseting.";
		view.reset();
		LOG(INFO) << "Reset successful.";
	}

	std::string version = aditof::getApiVersion();
	//my_log.AddLog("Preparing camera. Please wait...\n");
    LOG(INFO) << "Preparing camera. Please wait...\n";
	m_controller = std::make_shared<adicontroller::ADIController>(m_camerasList);

	view = std::make_shared<adiviewer::ADIView>(m_controller,
												"ToFViewer " + version);

	aditof::Status status = aditof::Status::OK;
	auto camera = getActiveCamera();//already initialized on constructor

	if (!camera) {
            LOG(ERROR) << "No cameras found!";
            return;
	}

    // user can pass any config.json stored anywhere in HW
    status = camera->setControl("initialization_config",
                                m_configFiles[configSelection].second);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not set the initialization config file!";
        return;
    }

	status = camera->initialize();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return;
    }

	//Parse config.json
	std::ifstream ifs(m_configFiles[configSelection].second);
	std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
	std::vector<std::pair<std::string, int32_t> > device_settings;
	cJSON* config_json = cJSON_Parse(content.c_str());
	
	if (config_json != NULL) {
		// Get GUI MAX_RANGE file location
		const cJSON* json_min_max_range = cJSON_GetObjectItemCaseSensitive(config_json, "MAX_RANGE");
		if (cJSON_IsString(json_min_max_range) && (json_min_max_range->valuestring != NULL)) {
			// Set Max range
			view->maxRange = std::stoi(json_min_max_range->valuestring);
		}
		//Get GUI MIN_RANGE file location
		json_min_max_range = cJSON_GetObjectItemCaseSensitive(config_json, "MIN_RANGE");
		if (cJSON_IsString(json_min_max_range) && (json_min_max_range->valuestring != NULL)) {
			// Set Min range
			view->minRange = std::stoi(json_min_max_range->valuestring);
		}
		//Get GUI AB_Max_RANGE file location
		json_min_max_range = cJSON_GetObjectItemCaseSensitive(config_json, "AB_Max_RANGE");
		if (cJSON_IsString(json_min_max_range) && (json_min_max_range->valuestring != NULL)) {
			// Set AB Max range
			view->maxABPixelValue = std::stoi(json_min_max_range->valuestring);
		}
		//Get available modes
		json_min_max_range = cJSON_GetObjectItemCaseSensitive(config_json, "modes");
		if (cJSON_IsString(json_min_max_range) && (json_min_max_range->valuestring != NULL)) {
			// Add to _cameraModes the available modes
            std::string cameraElements = static_cast<std::string>(json_min_max_range->valuestring);
            std::string delimiter = ",";
            size_t position = 0;
            std::string token = "";
			cameraElements.erase(std::remove(cameraElements.begin(),
                                               cameraElements.end(), ' '),
                                   cameraElements.end());//Cleanup the string to eliminate blank spaces.
            while ((position = cameraElements.find(delimiter)) != std::string::npos) {
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

    status = camera->setControl("powerUp", "call");
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not PowerUp camera!";
        return;
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

    aditof::CameraDetails camDetails;
    status = getActiveCamera()->getDetails(camDetails);
    int totalCaptures = camDetails.frameType.totalCaptures;	
	
	view->m_ctrl->m_recorder->m_frameDetails.totalCaptures = totalCaptures;

	// Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
	status = getActiveCamera()->start();
	if (status != aditof::Status::OK) {
		LOG(ERROR) << "Could not start camera!";
		return;
	}

	my_log.AddLog("Camera ready.\n");
	cameraWorkerDone = true;
}

bool ADIMainWindow::waitForCameraReady() {
	aditof::Status status = aditof::Status::OK;
	aditof::Frame temp_frame;

	aditof::FrameDetails frameDetails;
	// If the frames received are for previous MP mode, discard it and request after 2s( this value is app dependent)
	// Once got the proper frame, stop requesting
	while (1) {
		status = getActiveCamera()->requestFrame(&temp_frame);
		
		if (status == aditof::Status::BUSY) {
			LOG(INFO) << "Mode is changing. Waiting for new valid frames !";
		} else if (status != aditof::Status::OK) {
			LOG(ERROR) << "Could not request frame!";
			return false;
		} else {
			LOG(INFO) << "succesfully requested frame!";
			return true;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void ADIMainWindow::InitCCDCamera() {
	std::string version = aditof::getApiVersion();
	my_log.AddLog("Preparing camera. Please wait...\n");
	
	my_log.AddLog("Camera ready.\n");

	//TO DO: do we stil need this?

	//my_log.AddLog(sbuf->pubsetbuf);
}


void ADIMainWindow::PlayCCD(int modeSelect, int viewSelect) {
	ImGuiWindowFlags overlayFlags = /*ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |*/
		ImGuiWindowFlags_NoResize | /*ImGuiWindowFlags_AlwaysAutoResize |*/
		ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
		ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar| ImGuiWindowFlags_NoBringToFrontOnFocus;
	
	const bool imageIsHovered = ImGui::IsItemHovered();

    if (view->m_ctrl->hasCamera()) {
		// Mode switch or starup
		if (modeSelectChanged != modeSelect || 
			captureSeparateEnabled || !isPlaying) {
			if (modeSelectChanged != modeSelect) {
				view->m_ctrl->StopCapture();
			}

			prepareCamera(m_cameraModes[modeSelection].second);
			openGLCleanUp();
			initOpenGLIRTexture();
			initOpenGLDepthTexture();
			initOpenGLPointCloudTexture();	

			view->m_ctrl->StartCapture();
			view->m_ctrl->requestFrame();
			captureSeparateEnabled = false;
			modeSelectChanged = modeSelect;

			if (!waitForCameraReady()) {
				stopPlayCCD();
				return;
			}
        }
		else if (viewSelectionChanged != viewSelect) {
			viewSelectionChanged = viewSelect;
			openGLCleanUp();
			initOpenGLIRTexture();
			initOpenGLDepthTexture();
			initOpenGLPointCloudTexture();	
		}
    }

	switch (viewSelect) {
	case 0://Show Depth and IR in separate windows		     
		displayIR = true;
		displayDepth = checkCameraSetToReceiveContent("depth");	
		synchronizeDepthIRVideo();
		displayActiveBrightnessWindow(overlayFlags);
		if (displayDepth) {
			displayDepthWindow(overlayFlags);
		}
        displayInfoWindow(overlayFlags);
		break;
	case 1://Point Cloud Window
		displayDepth = checkCameraSetToReceiveContent("depth");
		if (displayDepth) {
			displayIR = false;			
			synchronizePointCloudVideo();
			displayPointCloudWindow(overlayFlags);
			displayDepthWindow(overlayFlags);
		} else {
            displayIR = true;
			synchronizeDepthIRVideo();	
            displayActiveBrightnessWindow(overlayFlags);			
		}
        displayInfoWindow(overlayFlags);
		break;
	default:
		break;
	}
}

void ADIMainWindow::displayInfoWindow(ImGuiWindowFlags overlayFlags) {
    if ((float)view->frameWidth == 0.0 && (float)(view->frameHeight == 0.0)) {
        return;
    }

    if (setTempWinPositionOnce) {
        rotationangleradians = 0;
        rotationangledegrees = 0;
        imagescale = (_isHighDPI) ? 1.0f : 0.5f;
        setTempWinPositionOnce = false;
    }

    float info_height = (_isHighDPI) ? 205.0f : 80.0f;

    if (displayIR)
        dictWinPosition["info"] = std::array<float, 4>(
            {dictWinPosition["ab"][0], offsetfromtop, 900, info_height});
    else
        dictWinPosition["info"] = std::array<float, 4>(
            {dictWinPosition["pc"][0], offsetfromtop, 900, info_height});

    setWindowPosition(dictWinPosition["info"][0],
                      dictWinPosition["info"][1]);
    setWindowSize(dictWinPosition["info"][2],
                  dictWinPosition["info"][3]);

    if (ImGui::Begin("Information Window", nullptr,
                     overlayFlags | ImGuiWindowFlags_NoTitleBar)) {
        //ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
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
        //ImGui::PopItemFlag();

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
                    view->frameHeight, (uint32_t)displayIRDimensions.x,
                    (uint32_t)displayIRDimensions.y);
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

    if (displayDataWindow(displayIRDimensions, size) == false)
        return;

    dictWinPosition["ab"] = std::array<float, 4>(
        {300,
         dictWinPosition["info"][1] + dictWinPosition["info"][3],
         size.x, size.y});
    setWindowPosition(dictWinPosition["ab"][0], dictWinPosition["ab"][1]);
    setWindowSize(dictWinPosition["ab"][2] + 40, dictWinPosition["ab"][3] + 40);

    if (ImGui::Begin("Active Brightness Window", nullptr, overlayFlags)) {
    }

	CaptureIRVideo();
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

    if (displayIR)
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["ab"][0] + dictWinPosition["ab"][2],
             dictWinPosition["info"][1] +
                 dictWinPosition["info"][3],
             size.x, size.y});
    else
        dictWinPosition["depth"] = std::array<float, 4>(
            {dictWinPosition["pc"][0] + dictWinPosition["pc"][2],
             dictWinPosition["info"][1] +
                 dictWinPosition["info"][3],
             size.x, size.y});
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
        {300,
         dictWinPosition["info"][1] + dictWinPosition["info"][3],
         size.x, size.y});

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
						ImColor(0, 180, 128),
						ImColor(0, 255, 255),
						ImColor(0, 0, 255)};  //Blue, closest

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
	depthDrawList->AddLine(ImVec2(p.x - CROSSHAIR_SIZE, p.y), ImVec2(p.x - 2, p.y), ImColor(255, 255, 255));
	depthDrawList->AddLine(ImVec2(p.x + CROSSHAIR_SIZE, p.y), ImVec2(p.x + 2, p.y), ImColor(255, 255, 255));
	depthDrawList->AddLine(ImVec2(p.x, p.y + CROSSHAIR_SIZE), ImVec2(p.x, p.y + 2), ImColor(255, 255, 255));
	depthDrawList->AddLine(ImVec2(p.x, p.y - CROSSHAIR_SIZE), ImVec2(p.x, p.y - 2), ImColor(255, 255, 255));	
	depthDrawList->AddText(ImGui::GetFont(), 24, position, ImColor(255, 255, 255, 255), "Test Standalone ImGui DrawList");
}

void ADIMainWindow::InitCaptureIRDepthVideo() {}

void ADIMainWindow::initOpenGLIRTexture() {
	/********************************************/
	//IR Texture
	GLuint ir_texture;
	glGenTextures(1, &ir_texture);
	glBindTexture(GL_TEXTURE_2D, ir_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	ir_video_texture = ir_texture;
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

void ADIMainWindow::preparePointCloudVertices(unsigned int& vbo, 
											  unsigned int& vao) {
	glGenVertexArrays(1, &vao);
	//Initialize Point Cloud Image
	glGenBuffers(1, &vbo);
	
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glBindVertexArray(vao);
	//Bind Point Cloud Image
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	//Pass on Image buffer here:
	glBufferData(GL_ARRAY_BUFFER, view->vertexArraySize, 
				view->normalized_vertices, GL_DYNAMIC_DRAW);//GL_STATIC_DRAW 
	
	//Image
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);//Our image buffer got bigger: [X, Y, Z, R, G, B]
	glEnableVertexAttribArray(0);//Enable [X, Y, Z]
	
	//Point Cloud Color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);//Enable [R, G, B]
	
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
	adiviewer::ADIShader vertexShader(GL_VERTEX_SHADER, pointCloudVertexShader); //Our vertices (whole image)
	adiviewer::ADIShader fragmentShader(GL_FRAGMENT_SHADER, pointCloudFragmentShader); //Color map
	view->pcShader.CreateProgram();
	view->pcShader.AttachShader(std::move(vertexShader));
	view->pcShader.AttachShader(std::move(fragmentShader));	
	view->pcShader.Link();

	//Get the model, view, and projection index
	view->modelIndex = glGetUniformLocation(view->pcShader.Id(), "model");
	view->viewIndex = glGetUniformLocation(view->pcShader.Id(), "view");
	view->projectionIndex = glGetUniformLocation(view->pcShader.Id(), "projection");
	
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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mainWindowWidth, mainWindowHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pointCloud_video_texture, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);	
}

void ADIMainWindow::synchronizeDepthIRVideo() {
	
	view->m_capturedFrame = view->m_ctrl->getFrame();	
	
	aditof::FrameDetails frameDetails;
	view->m_capturedFrame->getDetails(frameDetails);
	std::unique_lock<std::mutex> lock(view->m_frameCapturedMutex);
	if (displayIR) {
		view->m_irFrameAvailable = true;
	} else {
		view->m_irFrameAvailable = false;
		view->ir_video_data_8bit = nullptr;
	}
	if (displayDepth) {
		view->m_depthFrameAvailable = true;
	} else {
		view->m_depthFrameAvailable = false;
		view->depth_video_data_8bit = nullptr;
	}
	if (displayIR && displayDepth) {
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
		return view->m_waitKeyBarrier == view->numOfThreads;/*2*/; });
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
	view->m_barrierCv.wait(imshow_lock,
			[&]() { return view->m_waitKeyBarrier == view->numOfThreads; });
	view->m_waitKeyBarrier = 0;
	/*********************************/
}

void ADIMainWindow::GetHoveredImagePix(ImVec2& hoveredImagePixel, 
	ImVec2 imageStartPos, ImVec2 mousePos, ImVec2 displayDimensions) {
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

void ADIMainWindow::RenderInfoPane(ImVec2 hoveredImagePixel, uint16_t* currentImage, 
			int imageWidth, bool isHovered, ADI_Image_Format_t format, std::string units) {
	if (static_cast<int>(hoveredImagePixel.x) == static_cast<int>(InvalidHoveredPixel.x) &&
		static_cast<int>(hoveredImagePixel.y) == static_cast<int>(InvalidHoveredPixel.y)) {
		return;
	}

	ImGuiWindowFlags overlayFlags2 = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
		ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
		ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar;

	ImGui::SetNextWindowPos(ImGui::GetCursorScreenPos(), ImGuiCond_Always);
	ImGui::SetNextWindowBgAlpha(0.3f); // Transparent background
	uint16_t pixelValue = 0;

	if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_DEPTH16) {

		ImGui::Begin("dd", nullptr, overlayFlags2);

		if (hoveredImagePixel.x >= 0 && hoveredImagePixel.y >= 0) {
			pixelValue = currentImage[((int)hoveredImagePixel.y * (imageWidth)) + int(hoveredImagePixel.x)];//153280 is pixel value linear

			if (isHovered || ImGui::IsWindowHovered()) {
				ImGui::Text("Current pixel: %d, %d", int(hoveredImagePixel.x), int(hoveredImagePixel.y));
				ImGui::Text("Current pixel value: %d %s", pixelValue, units.c_str());
				//ImGui::Text("Temperature: %d %s", view->temperature_c, " C");//Not ready to implement yet.
			} else {
				ImGui::Text("Hover over the image to get pixel value");
			}
		}
		ImGui::End();
	} else if (format == ADI_Image_Format_t::ADI_IMAGE_FORMAT_IR16) {
		ImGui::Begin("dd", nullptr, overlayFlags2);

		if (hoveredImagePixel.x >= 0 && hoveredImagePixel.y >= 0) {
			pixelValue = currentImage[((int)hoveredImagePixel.y * (imageWidth)) + int(hoveredImagePixel.x)];//153280 is pixel value linear

			if (isHovered || ImGui::IsWindowHovered()) {
				ImGui::Text("Current pixel: %d, %d", int(hoveredImagePixel.x), int(hoveredImagePixel.y));
				ImGui::Text("Current pixel value: %d %s", pixelValue, units.c_str());
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

void ADIMainWindow::CaptureIRVideo() {
    if (view->ir_video_data_8bit != nullptr) {
        glBindTexture(GL_TEXTURE_2D, ir_video_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, view->frameWidth,
                     view->frameHeight, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     view->ir_video_data_8bit);
        glGenerateMipmap(GL_TEXTURE_2D);
        delete view->ir_video_data_8bit;

        ImVec2 _displayIRDimensions = displayIRDimensions;

        if (rotationangledegrees == 90 || rotationangledegrees == 270) {
            std::swap(_displayIRDimensions.x, _displayIRDimensions.y);
        }

        ImVec2 p = ImGui::GetCursorScreenPos();
        ImageRotated((ImTextureID)ir_video_texture,
                     ImVec2(dictWinPosition["ab"][2], dictWinPosition["ab"][3]),
                     ImVec2(_displayIRDimensions.x, _displayIRDimensions.y),
                     rotationangleradians);
        //ImGui::Image((void*)(intptr_t)ir_video_texture, displayIRDimensions);
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

void ADIMainWindow::processInputs(GLFWwindow* window) {
	//Sensitivity
	const float maxFov = 45.0f;
	float cameraSpeed = 2.5f * deltaTime;
	
	ImGuiIO &io = ImGui::GetIO();//Get mouse events

	if (ImGui::IsWindowHovered()) {
		if (io.MouseWheel)//Use mouse wheel to scroll up/down for zoom in/zoom out
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
	}else {
		mouseDown = false;
	}
	
	if (mouseDown) {
		const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
		// Normalize to the image start coordinates
		//
		mouseDownPos.x -= imageStartPos.x;
		mouseDownPos.y -= imageStartPos.y;

		const vec2 mouseDelta{ io.MouseDelta.x, io.MouseDelta.y };
		const vec2 displayDimensions{ static_cast<float>(mainWindowWidth), static_cast<float>(mainWindowHeight) };
		// Only count drags if they originated on the image
		//
		if (mouseDownPos.x >= 0.f && 
			mouseDownPos.x <= displayPointCloudDimensions.x && 
			mouseDownPos.y >= 0.f &&
			mouseDownPos.y <= displayPointCloudDimensions.y) {
			if (movementType == MouseMovementType::Rotation) {
				const ImVec2 imageStartPos = ImGui::GetCursorScreenPos();
				const vec2 mousePos{ io.MousePos.x + imageStartPos.x, io.MousePos.y + imageStartPos.y };
				vec2 lastMousePos;

				vec2_copy(lastMousePos, mousePos);
				vec2_sub(lastMousePos, mousePos, mouseDelta);

				quat newRotationQuat;
				GetArcballRotation(newRotationQuat, displayDimensions, lastMousePos, mousePos);

				mat4x4 newRotationMtx;
				mat4x4_from_quat(newRotationMtx, newRotationQuat);

				MatrixMultiply(m_model, newRotationMtx, m_model);
			}
			if (movementType == MouseMovementType::Translation) {
				cameraPos[0] -= mouseDelta[0] * TranslationSensitivity / (maxFov-fov+1);
				cameraPos[1] -= mouseDelta[1] * TranslationSensitivity / (maxFov-fov+1);
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

void ADIMainWindow::MapToArcball(vec3 out, const vec2 displayDimensions, const vec2 mousePos) {
	// Scale coords to (-1, 1) to simplify some of the math
	//
	vec2 scaledMousePos;
	for (int i = 0; i < 2; ++i) {
		scaledMousePos[i] = mousePos[i] * (1.0f / ((displayDimensions[i] - 1.0f) * 0.5f)) - 1.0f;
	}

	float lenSquared = scaledMousePos[0] * scaledMousePos[0] + scaledMousePos[1] * scaledMousePos[1];

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
		m_selectedDevice < 0 || m_selectedDevice >= view->m_ctrl->m_cameras.size()) {
		return nullptr;
	}
    return view->m_ctrl->m_cameras[m_selectedDevice];
}