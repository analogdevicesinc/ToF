/********************************************************************************/
/*                                                                              */
/* Copyright (c) Microsoft Corporation. All rights reserved.					*/
/*  Portions Copyright (c) 2020 Analog Devices Inc.								*/
/* Licensed under the MIT License.												*/
/*																				*/
/********************************************************************************/
#ifndef ADIMAINWINDOW_H
#define ADIMAINWINDOW_H

#include "ADIController.h"
#include "ADITypes.h"
#include "ADIView.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <aditof/system.h>
#include <linmath.h>
#include <stdio.h>

#include <unordered_map>
#define _USE_MATH_DEFINES
#include <array>
#include <math.h>

/**
* @brief Manage monitor resolution
*/
struct ADIViewerArgs {
    bool HighDpi = false;
};

/**
* @brief Logging Information
* @Usage:
*  static ExampleAppLog my_log;
*  my_log.AddLog("Hello %d world\n", 123);
*  my_log.Draw("title");
*/
struct AppLog {
    ImGuiTextBuffer Buf;
    ImGuiTextFilter Filter;
    ImVector<int>
        LineOffsets; // Index to lines offset. We maintain this with AddLog() calls, allowing us to have a random access on lines
    bool AutoScroll;
    bool ScrollToBottom;

    AppLog() {
        AutoScroll = true;
        ScrollToBottom = false;
        Clear();
    }

    void Clear() {
        Buf.clear();
        LineOffsets.clear();
        LineOffsets.push_back(0);
    }

    void AddLog(const char *fmt, ...) IM_FMTARGS(2) {
        int old_size = Buf.size();
        va_list args;
        va_start(args, fmt);
        Buf.appendfv(fmt, args);
        va_end(args);
        for (int new_size = Buf.size(); old_size < new_size; old_size++)
            if (Buf[old_size] == '\n')
                LineOffsets.push_back(old_size + 1);
        if (AutoScroll)
            ScrollToBottom = true;
    }

    void Draw(const char *title, bool *p_open = NULL) {
        if (!ImGui::Begin(title, p_open)) {
            ImGui::End();
            return;
        }

        // Options menu
        if (ImGui::BeginPopup("Options")) {
            if (ImGui::Checkbox("Auto-scroll", &AutoScroll))
                if (AutoScroll)
                    ScrollToBottom = true;
            ImGui::EndPopup();
        }

        // Main window
        if (ImGui::Button("Options"))
            ImGui::OpenPopup("Options");
        ImGui::SameLine();
        bool clear = ImGui::Button("Clear");
        ImGui::SameLine();
        bool copy = ImGui::Button("Copy");
        ImGui::SameLine();
        Filter.Draw("Filter", -100.0f);

        ImGui::Separator();
        ImGui::BeginChild("scrolling", ImVec2(0, 0), false,
                          ImGuiWindowFlags_HorizontalScrollbar);

        if (clear)
            Clear();
        if (copy)
            ImGui::LogToClipboard();

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
        const char *buf = Buf.begin();
        const char *buf_end = Buf.end();
        if (Filter.IsActive()) {
            // In this example we don't use the clipper when Filter is enabled.
            // This is because we don't have a random access on the result on our filter.
            // A real application processing logs with ten of thousands of entries may want to store the result of search/filter.
            // especially if the filtering function is not trivial (e.g. reg-exp).
            for (int line_no = 0; line_no < LineOffsets.Size; line_no++) {
                const char *line_start = buf + LineOffsets[line_no];
                const char *line_end =
                    (line_no + 1 < LineOffsets.Size)
                        ? (buf + LineOffsets[line_no + 1] - 1)
                        : buf_end;
                if (Filter.PassFilter(line_start, line_end)) {
                    if (line_start[0] == 'W' ||
                        line_start[0] ==
                            'w') { // If line[0] starts with a W, that means it is a Warning. Display it as Yellow
                        ImGui::PushStyleColor(ImGuiCol_Text,
                                              ImVec4(1.0f, 1.0f, 0.0f,
                                                     1.0f)); // R, G, B, Y
                        ImGui::TextUnformatted(line_start, line_end);
                        ImGui::PopStyleColor();
                    } else if (
                        line_start[0] == 'E' ||
                        line_start[0] ==
                            'e') { // If line[0] starts with an E, this is an Error. Display it as Red
                        ImGui::PushStyleColor(ImGuiCol_Text,
                                              ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                        ImGui::TextUnformatted(line_start, line_end);
                        ImGui::PopStyleColor();
                    } else { // Display as regular white text. This should be Information type.
                        ImGui::TextUnformatted(line_start, line_end);
                    }
                }
            }
        } else {
            // The simplest and easy way to display the entire buffer:
            //   ImGui::TextUnformatted(buf_begin, buf_end);
            // And it'll just work. TextUnformatted() has specialization for large blob of text and will fast-forward to skip non-visible lines.
            // Here we instead demonstrate using the clipper to only process lines that are within the visible area.
            // If you have tens of thousands of items and their processing cost is non-negligible, coarse clipping them on your side is recommended.
            // Using ImGuiListClipper requires A) random access into your data, and B) items all being the  same height,
            // both of which we can handle since we an array pointing to the beginning of each line of text.
            // When using the filter (in the block of code above) we don't have random access into the data to display anymore, which is why we don't use the clipper.
            // Storing or skimming through the search result would make it possible (and would be recommended if you want to search through tens of thousands of entries)
            ImGuiListClipper clipper;
            clipper.Begin(LineOffsets.Size);
            while (clipper.Step()) {
                for (int line_no = clipper.DisplayStart;
                     line_no < clipper.DisplayEnd; line_no++) {
                    const char *line_start = buf + LineOffsets[line_no];
                    const char *line_end =
                        (line_no + 1 < LineOffsets.Size)
                            ? (buf + LineOffsets[line_no + 1] - 1)
                            : buf_end;
                    if (line_start[0] == 'W' ||
                        line_start[0] ==
                            'w') { // If line[0] starts with a W, that means it is a Warning. Display it as Yellow
                        ImGui::PushStyleColor(
                            ImGuiCol_Text,
                            ImVec4(1.0f, 1.0f, 0.0f, 1.0f)); // R, G, B, Y
                        ImGui::TextUnformatted(line_start, line_end);
                        ImGui::PopStyleColor();
                    } else if (
                        line_start[0] == 'E' ||
                        line_start[0] ==
                            'e') { // If line[0] starts with an E, this is an Error. Display it as Red
                        ImGui::PushStyleColor(ImGuiCol_Text,
                                              ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                        ImGui::TextUnformatted(line_start, line_end);
                        ImGui::PopStyleColor();
                    } else { // Display as regular white text. This should be Information type.
                        ImGui::TextUnformatted(line_start, line_end);
                    }
                }
            }
            clipper.End();
        }
        ImGui::PopStyleVar();

        if (ScrollToBottom)
            ImGui::SetScrollHereY(1.0f);
        ScrollToBottom = false;
        ImGui::EndChild();
        ImGui::End();
    }
};

namespace adiMainWindow {

class ADIMainWindow {
  private:
    const float OFFSETFROMTOPOFGUI = 38;
    const float OFFSETFROMLEFT = 38;
    const float INITIALRAD = 0.0f;
    const uint32_t INITIALDEG = 0;
    const float NORMALDPISCALAR = 1.0f;
    const float HIGHDPISCALAR = 2.0f;
    const std::string DEFAULT_TOOLS_CONFIG_FILENAME = "tof-tools.config";

  public:
    /**
		* @brief Main window constructor
		*/
    ADIMainWindow();

    /**
		* @brief Main window destructor
		*/
    ~ADIMainWindow();

    /**
		* @brief Start Main GUI window.
		*        Using Dear ImGUIr
		*/
    bool startImGUI(const ADIViewerArgs &args);

    /**
		* @brief Render Main Window after a successful imGUI setup
		*/
    void render();

    /**
		* @brief Modify Screen DPI
		*/
    void setDpi();

    bool _isHighDPI = false;
    float dpiScaleFactor = HIGHDPISCALAR;

    std::thread initCameraWorker;
    bool cameraWorkerDone = false;

    /**
		* @brief	CPU percentage Usage
		*/
    double getCurrentValue();
    int recordingSeconds = 5; //Default value

    /**
		* @brief Stops Playback
		*/
    void stopPlayback();

    /**
		* @brief Deletes all buffers and
		*        frees up memory
		*/
    void openGLCleanUp();

    //Point Cloud
    mat4x4 m_view;
    mat4x4 m_projection;
    mat4x4 m_model;
    vec3 cameraPos = {0.0f, 0.0f, 3.0f};
    vec3 cameraFront = {0.0f, 0.0f, -1.0f};
    vec3 cameraUp = {0.0f, 1.0f, 0.0f};
    vec3 cameraPos_Front;
    float deltaTime = 0;
    float lastFrame = 0;
    float fov = 8.0f; //field of view angle in degrees.
    // yaw is initialized to -90.0 degrees since a yaw of 0.0
    //results in a direction vector pointing to the right so
    //we initially rotate a bit to the left.
    float yaw = -90.0f;
    float pitch = 0.0f;
    bool firstMouse = true;
    float lastX = 1280.0f;
    float lastY = 720.0;
    float TranslationSensitivity = 0.03f;
    int pointSize = 1;
    unsigned int framebuffer;
    bool mouseDown = false;
    bool m_saveBinaryFormatTmp = false;

    /**
		* @brief Virtual Sphere
		* Imagine a virtual sphere of displayDimensions size were drawn on the screen.
		* Returns a quaternion representing the rotation that sphere would undergo if you took the point
		* on that sphere at startPos and rotated it to endPos (i.e. an "arcball" camera).
		*/
    void GetArcballRotation(quat rotation, const vec2 displayDimensions,
                            const vec2 startPos, const vec2 endPos);

    /**
		* @brief Map XY coordinates to a virtual sphere, which we use
		*		 for rotation calculations
		*/
    void MapToArcball(vec3 out, const vec2 displayDimensions,
                      const vec2 mousePos);

    /**
		* @brief Version of mat4x4_mul that doesn't modify its non-result inputs (i.e. a, b)
        */
    void MatrixMultiply(mat4x4 out, mat4x4 a, mat4x4 b);

    // The type of movement that a mouse movement should be interpreted as (if any)
    //
    enum class MouseMovementType { None, Rotation, Translation };

    AppLog *getLog() { return &my_log; }
    const std::string INIT_LOG_WARNING =
        "WARNING: Logging before InitGoogleLogging() is written to "
        "STDERR\n";

  private:
    float rotationangleradians = INITIALRAD;
    uint32_t rotationangledegrees = INITIALDEG;
    const float offsetfromtop = OFFSETFROMTOPOFGUI;
    const float offsetfromleft = OFFSETFROMLEFT;
    std::unordered_map<std::string, std::array<float, 4>> dictWinPosition;
    std::shared_ptr<adiviewer::ADIView> view = nullptr;
    AppLog my_log;

    int frameCounter = 0;
    int fps = 0;

    /**
		* @brief Rotation of an ImGui texture.
		*/
    void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size,
                      float angle);

    /**
		* @brief Vector operations to support rotation.
		*/
    ImVec2 ImRotate(const ImVec2 &v, float cos_a, float sin_a);

    /**
		* @brief Create the necessary OpenGL vertex attributes and buffers based on
		* 		 current vertices extracted from XYZ data
		*/
    void preparePointCloudVertices(unsigned int &vbo, unsigned int &vao);

    /**
		* @brief Resets to camera default position, as well as vertex point size
		*/
    void pointCloudReset();

    /**
		* @brief Process keyboard and mouse
		*        inputs to move Point Cloud
		*        image
		*/
    void processInputs(GLFWwindow *window);

    /**
		* @brief Main Menu
		*/
    void showMainMenu();

    /**
		* @brief	Open Device menu will give you the information of
		*			the supported camera. It will also fetch for
		*			any other USB devices and only recognize the
		*			supported device(s), otherwise it will just
		*			"wait" for a new plug-and-play device and
		*			refresh options
		*/
    void showDeviceMenu();

    /**
     * @brief Show the Analog Devices, Inc log
    */
    void showLogoWindow();

    /**
		* @brief Displays Log information
		*/
    void showLogWindow(bool *p_open);

    /**
		* @brief Will poll the USB interface to look
		*        for supported devices
		*/
    void RefreshDevices();

    /**
		* @brief Initialize CMOS Camera
		*/
    void InitCamera();

    /**
		* @brief				Will Play ADI CCD depending on
		*						mode and view settings
		* @param modeSelect		Camera mode selection
		* @param viewSelect		Window view selection:
		*						- Depth and IR
		*						- Blended Depth with IR
		*/
    void PlayCCD(int modeSelect, int viewSelect);

    /**
		* @brief Playback recorded video
		*/
    void PlayRecorded();

    /**
		* @brief Displays Playback menu
		*/
    void showPlaybackMenu();

    /**
		* @brief Initializes OpenGL and IR + Depth settings
		*/
    void InitCaptureIRDepthVideo();

    /**
		* @brief Renders Depth OpenGL images
		*/
    void CaptureDepthVideo();

    /**
		* @brief Renders IR OpenGL images
		*/
    void CaptureIRVideo();

    /**
		* @brief Renders Point Cloud OpenGL images
		*/
    void CapturePointCloudVideo();

    /**
		* @brief Initializes OpenGL for IR texture
		*/
    void initOpenGLIRTexture();

    /**
		* @brief Initializes OpenGL for Depth texture
		*/
    void initOpenGLDepthTexture();

    /**
		* @brief Initializes OpenGL for Point
		*        Cloud texture
		*/
    void initOpenGLPointCloudTexture();

    /**
		* @brief Stops playing CCD Camera
		*/
    void stopPlayCCD();

    /**
		* @brief Sets the threads to get both
		*        IR and Depth images
		*/
    void synchronizeDepthIRVideo();

    /**
		* @brief Sets the thread to get
		*        Point Cloud image
		*/
    void synchronizePointCloudVideo();

    /**
		* @brief Displays pixel information while mouse hovers over
		*/
    void RenderInfoPane(ImVec2 hoveredImagePixel, uint16_t *currentImage,
                        int imageWidth, bool isHovered,
                        ADI_Image_Format_t format, std::string units);

    /**
		* @brief Returns the hovered pixel value
		*/
    void GetHoveredImagePix(ImVec2 &hoveredImagePixel, ImVec2 imageStartPos,
                            ImVec2 mousePos, ImVec2 displayDimensions);
    int m_selectedDevice = -1;
    std::vector<std::pair<int, std::string>> m_connectedDevices;

    /**
		* @brief Returns current application path
		*/
    std::string getCurrentPath();

    /**
		* @brief			Prepares the camera with the selected mode
		* @param	mode	Camera mode
		*/
    void prepareCamera(std::string mode);

    /**
		* @brief Displays the Information Window
		*/
    void displayInfoWindow(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Displays AB Window
		*/
    void displayActiveBrightnessWindow(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Displays Depth Window
		*/
    void displayDepthWindow(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Displays Point Cloud Window
		*/
    void displayPointCloudWindow(ImGuiWindowFlags overlayFlags);

    /**
		 * @brief Checks if the frame type set in camera has a certain
		 *        content. (e.g. 'depth', 'ir', 'xyz', etc.)
		 */
    bool checkCameraSetToReceiveContent(const std::string &contentType);

    /**
     * @brief Computes streaming fps
    */
    void computeFPS(int &fps);

    /**
         * @brief Return the current selected camera object
         */
    std::shared_ptr<aditof::Camera> getActiveCamera();

    aditof::System m_system;
    std::vector<std::shared_ptr<aditof::Camera>> m_camerasList;

    std::shared_ptr<adicontroller::ADIController> m_controller;

    bool m_skipNetworkCameras;
    std::string m_cameraIp;
    std::vector<std::pair<int, std::string>> m_configFiles;
    int configSelection = 0;

    std::string modes[3] = {"near", "medium", "far"};
    bool _usesExternalModeDefinition = false;
    bool captureSeparateEnabled = true;
    bool captureBlendedEnabled = true;
    const ImVec2 InvalidHoveredPixel = ImVec2(-1, -1);
    std::vector<std::string> _cameraModes;
    std::vector<std::pair<int, std::string>> m_cameraModes;

    const ImVec2 depthWinSize = ImVec2(0, 0);
    ImVec2 sourceDepthImageDimensions;
    ImVec2 displayDepthDimensions;
    ImVec2 sourceIRImageDimensions;
    ImVec2 displayIRDimensions;
    ImVec2 sourcePointCloudImageDimensions;
    ImVec2 displayPointCloudDimensions;

    size_t dephtWinCtr = 0;

    //imGUI
    GLFWwindow *window;
    int mainWindowHeight;
    int mainWindowWidth;
    bool showIRWindow = false;
    bool showDepthWindow = false;
    int modeSelection = 0;
    int modeSelectChanged = 0; //flag when changed
    bool isPlaying = false;
    bool isRecording = false;
    bool isPlayRecorded = false;
    bool isPlayRecordPaused = false;
    int viewSelection = 0;
    int viewSelectionChanged = 0; //flag when changed
    bool cameraOptionsTreeEnabled = true;
    bool pointCloudEnable = true;

    /**
		* @brief Set any window a specific position
		*/
    void setWindowPosition(float x, float y);

    /**
		* @brief Convert degrees into radians
		*/
    float radians(float degrees);

    /**
		* @brief Set any window a specific size
		*/
    void setWindowSize(float width, float height);

    /**
		* @brief Creates a Depth color bar that gives
		*        a brief idea of mapped distance
		*/
    void createColorBar(ImVec2 position, ImVec2 size);

    /**
		* @brief Common pre-display for GUI windows
		*/
    bool displayDataWindow(ImVec2 &displayUpdate, ImVec2 &size);

    float imagescale = 1.0f;
    const float imagesizemax = 516.0f;
    unsigned int ir_video_texture = 0;
    unsigned int depth_video_texture = 0;
    unsigned int pointCloud_video_texture = 0;
    bool setTempWinPositionOnce = true;
    bool setIRWinPositionOnce = true;
    bool setDepthWinPositionOnce = true;
    bool setLogWinPositionOnce = true;
    bool setPointCloudPositionOnce = true;
    bool _isOpenDevice;
    float customColorPlay = 0.4f;
    float customColorStop = 0.0f;
    float customColorSave = 0.3f;
    float customColorOpen = 0.2f;
    float customColorPause = 0.1f;
    bool displayIR = true;
    bool displayDepth = true;
    bool displayPointCloud = false;
    bool displayTemp = true;
    int rawSeeker = 0;

    //Logging
    char buffer[1024];
    FILE *stream;
    FILE *input;
};
} // namespace adiMainWindow
#endif //ADIMAINWINDOW_H