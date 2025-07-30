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
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_opengl3_loader.h"
#include "imgui_toggle.h"
#include "implot.h"
#include <GLFW/glfw3.h>
#include <aditof/system.h>
#include <linmath.h>
#include <stdio.h>

#include <unordered_map>
#define _USE_MATH_DEFINES
#include <array>
#include <chrono>
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
*  static ExampleAppLog m_log;
*  m_log.AddLog("Hello %d world\n", 123);
*  m_log.Draw("title");
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

    void Draw(const char *title, bool *p_open = NULL,
              ImGuiWindowFlags windowFlags = NULL) {
        if (!ImGui::Begin(title, p_open, windowFlags)) {
            ImGui::End();
            return;
        }

        // Options menu: TODO - fix me
        /*
        if (ImGui::BeginPopup("Options")) {
            if (ImGui::Checkbox("Auto-scroll", &AutoScroll))
                if (AutoScroll)
                    ScrollToBottom = true;
            ImGui::EndPopup();
        }

        if (ImGui::Button("Options"))
            ImGui::OpenPopup("Options");
        */

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

struct Rect {
    float x;
    float y;
    float width;
    float height;
};

enum class MouseMovementType { None, Rotation, Translation };

namespace adiMainWindow {

class ADIMainWindow {
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
    bool StartImGUI(const ADIViewerArgs &args);

    /**
		* @brief Render Main Window after a successful imGUI setup
		*/
    void Render();

  private:
    /**
		* @brief Modify Screen DPI
		*/
    void SetDpi();

    /**
		* @brief Stops Playback
		*/
    void StopPlayback();

    /**
		* @brief Deletes all buffers and
		*        frees up memory
		*/
  
    void OpenGLCleanUp();

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
    int32_t PreparePointCloudVertices(GLuint &vbo, GLuint&vao);

    /**
		* @brief Resets to camera default position, as well as vertex point size
		*/
    void PointCloudReset();

    /**
		* @brief Process keyboard and mouse
		*        inputs to move Point Cloud
		*        image
		*/
    void ProcessInputs(GLFWwindow *window);

    void HandleInterruptCallback();

    /**
		* @brief Main Menu
		*/
    void ShowMainMenu();

    void ShowLoadAdsdParamsMenu();
    void ShowSaveAdsdParamsMenu();

    /**
		* @brief	Open Device menu will give you the information of
		*			the supported camera. It will also fetch for
		*			any other USB devices and only recognize the
		*			supported device(s), otherwise it will just
		*			"wait" for a new plug-and-play device and
		*			refresh options
		*/
    void ShowDeviceMenu();

    /**
     * @brief Show the Analog Devices, Inc log
    */
    void ShowLogoWindow();

    /**
		* @brief Displays Log information
		*/
    void ShowLogWindow(bool *p_open);

    void ShowIniWindow(bool showModify = true);

    /**
		* @brief Will poll the USB interface to look
		*        for supported devices
		*/
    void RefreshDevices();

    /**
		* @brief Initialize CMOS Camera
		*/
    void InitCamera(std::string filePath);

    /**
		* @brief				Will Play ADI CCD depending on
		*						mode and view settings
		* @param modeSelect		Camera mode selection
		* @param viewSelect		Window view selection:
		*						- Depth and AB
		*						- Blended Depth with AB
		*/
    void CameraPlay(int modeSelect, int viewSelect);

    /**
		* @brief Initializes OpenGL and AB + Depth settings
		*/
    void InitCaptureABDepthVideo();

    /**
		* @brief Renders Depth OpenGL images
		*/
    void CaptureDepthVideo();

    void DepthLinePlot(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Renders AB OpenGL images
		*/
    void CaptureABVideo();

    /**
		* @brief Renders Point Cloud OpenGL images
		*/
    void CapturePointCloudVideo();

    /**
		* @brief Initializes OpenGL for AB texture
		*/
    void InitOpenGLABTexture();

    /**
		* @brief Initializes OpenGL for Depth texture
		*/
    void InitOpenGLDepthTexture();

    /**
		* @brief Initializes OpenGL for Point
		*        Cloud texture
		*/
    void InitOpenGLPointCloudTexture();

    /**
		* @brief Stops playing CCD Camera
		*/
    void CameraStop();

    int32_t synchronizeVideo(std::shared_ptr<aditof::Frame>& frame);

    /**
		* @brief Displays pixel information while mouse hovers over
		*/
    void RenderFrameHoverInfo(ImVec2 hoveredImagePixel, uint16_t *currentImage,
                        int imageWidth, bool isHovered,
                        ADI_Image_Format_t format, std::string units);

    /**
		* @brief Returns the hovered pixel value
		*/
    void GetHoveredImagePix(ImVec2 &hoveredImagePixel, ImVec2 imageStartPos,
                            ImVec2 mousePos,
                            ImVec2 display_depth_dimensions, ImVec2 source_depth_image_dimensions);

    /**
		* @brief Returns current application path
		*/
    std::string getCurrentPath();

    /**
		* @brief			Prepares the camera with the selected mode
		* @param	mode	Camera mode
		*/
    void PrepareCamera(uint8_t mode);

    /**
		* @brief Displays the Information Window
		*/
    void DisplayInfoWindow(ImGuiWindowFlags overlayFlags, bool diverging);
    float DisplayFrameWindow(ImVec2 windowSize, ImVec2 &displayUpdate,
                            ImVec2 &size);
    void DisplayControlWindow(ImGuiWindowFlags overlayFlags, bool haveAB, bool haveDepth, bool haveXYZ);

    /**
		* @brief Displays AB Window
		*/
    void DisplayActiveBrightnessWindow(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Displays Depth Window
		*/
    void DisplayDepthWindow(ImGuiWindowFlags overlayFlags);

    /**
		* @brief Displays Point Cloud Window
		*/
    void DisplayPointCloudWindow(ImGuiWindowFlags overlayFlags);

    /**
     * @brief print out warning message in popup window if ini param is out of valid range
    */
    void IniParamWarn(std::string variable, std::string validVal);
    bool EntryInt32_t(const char* label, int32_t& input, const int32_t min, const int32_t max);

    void CustomizeMenus();

    /**
     * @brief Return the current selected camera object
    */
    std::shared_ptr<aditof::Camera> GetActiveCamera();

    const uint32_t MAX_FRAME_RATE = 25;
    uint32_t m_max_frame_rate = MAX_FRAME_RATE;

    /**
	* @brief Set any window a specific position
	*/
    void SetWindowPosition(float x, float y);

    /**
		* @brief Convert degrees into radians
		*/
    float Radians(float degrees);

    /**
		* @brief Set any window a specific size
		*/
    void SetWindowSize(float width, float height);

    /**
		* @brief Creates a Depth color bar that gives
		*        a brief idea of mapped distance
		*/
    void CreateColorBar(ImVec2 position, ImVec2 size);

    /**
		* @brief Common pre-display for GUI windows
		*/
    bool DisplayDataWindow(ImVec2 &displayUpdate, ImVec2 &size);
    void DisplayHelp();
    void DrawColoredLabel(const char *fmt, ...);
    void DrawBarLabel(const char *fmt, ...);
    void NewLine(float spacing);
    void ShowStartWizard();
    bool SaveAllFramesUpdate();
    bool cameraButton(std::string& baseFileName);
    int32_t SaveTextureAsJPEG(const char* filename, GLuint textureID, uint32_t width, uint32_t height);
    int32_t SaveConfidenceAsJPEG(const char* filename, const std::shared_ptr<aditof::Frame> frame, uint32_t width, uint32_t height);
    void SavePointCloudPLYBinary(const char* filename, const float* points, size_t num_points);
    void SaveMetaAsTxt(const char* filename, std::shared_ptr<aditof::Frame> frame);
    ImFont* LoadFont(const unsigned char* ext_font, const unsigned int ext_font_len, const float size);
    void centreWindow(float width, float height);
    float WindowCalcX(Rect w, float buffer = 0.0f) {
        return w.x + w.width + buffer;
    }
    float WindowCalcY(Rect w, float buffer = 0.0f) {
        return w.y + w.height + buffer;
    }
    void Spinner(const char* label, float radius, int thickness, ImU32 color);

    bool m_isWorking = false;
	bool getIsWorking() const { return m_isWorking; }
	void setIsWorking(bool isWorking) { m_isWorking = isWorking; }
    void GetYawPitchRoll(float& yaw, float& pitch, float& roll);

    // The type of movement that a mouse movement should be interpreted as (if any)
    //
    AppLog *GetLog() { return &m_log; }

    // Constants
    const std::string INIT_LOG_WARNING =
        "WARNING: Logging before InitGoogleLogging() is written to "
        "STDERR\n";
    const int32_t MAX_RECORD_TIME = 60;
    const float OFFSETFROMTOPOFGUI = 38;
    const float OFFSETFROMLEFT = 38;
    const float INITIALRAD = 0.0f;
    const uint32_t INITIALDEG = 0;
    const float NORMALDPISCALAR = 1.0f;
    const float HIGHDPISCALAR = 2.0f;
    const float offsetfromtop = OFFSETFROMTOPOFGUI;
    const float offsetfromleft = OFFSETFROMLEFT;
    const std::string DEFAULT_TOOLS_CONFIG_FILENAME = "tof-tools.config";
    const float m_custom_color_play = 0.4f;
    const float m_custom_color_stop = 0.0f;
    const float m_custom_color_save = 0.3f;
    const float m_custom_color_open = 0.2f;
    const float m_custom_color_pause = 0.1f;
    const ImVec2 m_invalid_hovered_pixel = ImVec2(-1, -1);
    const uint32_t PREVIEW_FRAME_RATE = 3;

    //Point Cloud
    mat4x4 m_view_mat;
    mat4x4 m_projection_mat;
    mat4x4 m_model_mat;
    vec3 m_camera_position_vec = {0.0f, 0.0f, 3.0f};
    vec3 m_camera_front_vec = {0.0f, 0.0f, -1.0f};
    vec3 m_camera_up_vec = {0.0f, 1.0f, 0.0f};
    vec3 m_camera_position_front_vec;
    float m_delta_time = 0.1f;
    float m_field_of_view = 8.0f;
    float m_translation_sensitivity = 0.03f;
    int32_t m_point_size = 1;
    GLuint m_gl_pc_colourTex;
    GLuint m_gl_pc_depthTex;
    bool m_mouse_down = false;
    bool m_saveBinaryFormatTmp = false;
    float m_tof_image_pos_y;
    float m_dpi_scale_factor = HIGHDPISCALAR;
    std::thread initCameraWorker;
    bool m_cameraWorkerDone = false;
    aditof::System m_system;
    std::vector<std::shared_ptr<aditof::Camera>> m_cameras_list;
    bool m_focused_once = false;
    bool m_skip_network_cameras;
    std::string m_cameraIp;
    std::vector<std::pair<int, std::string>> m_configFiles;
    int32_t m_config_selection = 0;
    bool m_uses_external_mode_definition = false;
    bool m_capture_separate_enabled = true;
    std::vector<uint8_t> _cameraModes;
    std::vector<std::pair<int, uint8_t>> m_cameraModes;
    std::vector<std::pair<int, std::string>> m_cameraModesDropDown;
    std::unordered_map<uint16_t, std::string> m_cameraModesLookup;
    bool m_recordingActive = false;
    ImVec2 m_source_depth_image_dimensions;
    ImVec2 m_display_depth_dimensions;
    ImVec2 m_source_ab_image_dimensions;
    ImVec2 m_display_ab_dimensions;
    ImVec2 m_source_point_cloud_image_dimensions;
    ImVec2 m_display_point_cloud_dimensions;
    //imGUI
    GLFWwindow *window;
    int32_t m_main_window_height;
    int32_t m_main_window_width;
    int32_t m_mode_selection = 0;
    int32_t m_mode_select_changed = 0; //flag when changed
    bool m_is_playing = false;
    int32_t m_view_selection = 0;
    int32_t m_view_selection_changed = 0; //flag when changed
    float m_image_scale = 1.0f;
    const float imagesizemax = 516.0f;
    uint32_t m_gl_ab_video_texture = 0;
    uint32_t m_gl_depth_video_texture = 0;
    uint32_t m_gl_pointcloud_video_texture = 0;
    bool m_set_temp_win_position_once = true;
    bool m_set_ab_win_position_once = true;
    bool m_set_depth_win_position_once = true;
    bool m_set_log_win_position_once = true;
    bool m_set_point_cloud_position_once = true;
    bool m_is_open_device = false;

    //Logging
    char m_buffer[1024];
    FILE *m_file_stream;
    FILE *m_file_input;
    float rotationangleradians = INITIALRAD;
    uint32_t rotationangledegrees = INITIALDEG;

    std::unordered_map<std::string, Rect> m_dict_win_position;
    Rect *m_xyz_position;
    Rect *m_ab_position;
    Rect *m_depth_position;
    int16_t m_frame_window_position_state;
    std::shared_ptr<adiviewer::ADIView> m_view_instance = nullptr;
    AppLog m_log;

    bool m_callback_initialized = false;
    bool m_network_link_test = false;
    std::string m_ip_suffix;

    bool m_off_line = false;
    uint32_t m_off_line_frame_index = 0;
    int32_t m_frame_counter = 0;
    uint16_t m_fps_expected = 0;
    uint32_t m_fps_frame_received = 0;
    int32_t m_selected_device_index = -1;
    std::vector<std::pair<int, std::string>> m_connected_devices;

    uint8_t m_last_mode = -1;
    bool m_enable_preview = false;

    std::map<std::string, std::string> m_ini_params;
    std::map<std::string, std::string> m_modified_ini_params;
    bool m_use_modified_ini_params;

    bool m_offline_save_all_frames;
    std::vector<float> m_depth_line_values;
    std::vector<std::pair<float, float>> m_depthLine;
    bool m_flash_main_window = false;
    int32_t m_user_frame_rate = 0;
    std::atomic<bool> m_offline_change_frame;
	std::string m_base_file_name;
};
} // namespace adiMainWindow
#endif //ADIMAINWINDOW_H
