#include "ADIMainWindow.h"
#include "aditof/system.h"
#include "aditof/version.h"
#include "aditof/status_definitions.h"
#include "ADIOpenFile.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif

#include <iostream>

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

using namespace adiMainWindow;

void ADIMainWindow::InitCamera(std::string filePath) {
    if (m_view_instance != NULL) { //Reset current imager
        LOG(INFO) << "Imager is reseting.";
        m_view_instance.reset();
        LOG(INFO) << "Reset successful.";
    }

    std::string version = aditof::getApiVersion();
    LOG(INFO) << "Preparing camera. Please wait...\n";
    m_view_instance = std::make_shared<adiviewer::ADIView>(
        std::make_shared<adicontroller::ADIController>(m_cameras_list),
        "ToFViewer " + version);

    if (!m_off_line) {
        m_cameras_list.clear();
        _cameraModes.clear();
        m_cameraModesDropDown.clear();
        m_cameraModesLookup.clear();
    }

    aditof::Status status = aditof::Status::OK;
    auto camera = GetActiveCamera(); //already initialized on constructor

    if (!camera) {
        LOG(ERROR) << "No cameras found!";
        return;
    }

    status = camera->initialize("");
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return;
    }

    if (!m_off_line) {

        aditof::CameraDetails cameraDetails;
        camera->getDetails(cameraDetails);

        LOG(INFO) << "SD card image version: "
                  << cameraDetails.sdCardImageVersion;
        LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
        LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

        camera->getAvailableModes(_cameraModes);
        sort(_cameraModes.begin(), _cameraModes.end());

        for (int i = 0; i < _cameraModes.size(); ++i) {
            aditof::DepthSensorModeDetails modeDetails;

            auto sensor = camera->getSensor();
            sensor->getModeDetails(_cameraModes.at(i), modeDetails);

            std::string s = std::to_string(_cameraModes.at(i));
            s = s + ":" + std::to_string(modeDetails.baseResolutionWidth) +
                "x" + std::to_string(modeDetails.baseResolutionHeight) + ",";
            if (!modeDetails.isPCM) {
                std::string append = (modeDetails.numberOfPhases == 2) ? "Short Range" : "Long Range";
                s = s + append;
            } else {
                s = s + "PCM";
            }
            m_cameraModesDropDown.emplace_back(modeDetails.modeNumber, s);
            m_cameraModesLookup[modeDetails.modeNumber] = s;
        }

        for (int i = 0; i < _cameraModes.size(); i++) {
            m_cameraModes.emplace_back(i, _cameraModes.at(i));
        }
    } else {
        // PRB25
        //camera->startPlayback(filePath);
    }
    m_cameraWorkerDone = true;
    m_is_open_device = true;
}

void ADIMainWindow::PrepareCamera(uint8_t mode) {
    aditof::Status status = aditof::Status::OK;
    std::vector<aditof::FrameDetails> frameTypes;

    status = GetActiveCamera()->setMode(mode);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return;
    }
#if 0 //Andre
    if (!m_off_line) {
        status = GetActiveCamera()->adsd3500SetFrameRate(m_user_frame_rate);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Could not set user frame rate!";
            return;
        }
    }
#endif 
    if (mode == m_last_mode) {
        if (!m_modified_ini_params.empty()) {
            if (m_use_modified_ini_params) {
                status = GetActiveCamera()->setFrameProcessParams(
                    m_modified_ini_params, mode);
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Could not set ini params";
                } else {
                    LOG(INFO) << "Using user defined ini parameters.";
                    m_use_modified_ini_params = false;
                    m_modified_ini_params.clear();
                }
            }
        }
    } else {
        m_use_modified_ini_params = false;
        m_ini_params.clear();
        m_modified_ini_params.clear();
        m_last_mode = mode;
    }

    aditof::CameraDetails camDetails;
    status = GetActiveCamera()->getDetails(camDetails);
    int totalCaptures = camDetails.frameType.totalCaptures;

    status = GetActiveCamera()->adsd3500GetFrameRate(m_fps_expected);

    if (m_enable_preview) {
        m_view_instance->m_ctrl->setPreviewRate(m_fps_expected, PREVIEW_FRAME_RATE);
    }
    else {
        m_view_instance->m_ctrl->setPreviewRate(m_fps_expected, m_fps_expected);
    }

    if (!m_view_instance->getUserABMaxState()) {
        std::string value;
        GetActiveCamera()->getSensor()->getControl("abBits", value);
        m_view_instance->setABMaxRange(value);
    }

    // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
    status = GetActiveCamera()->start();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return;
    }

    LOG(INFO) << "Camera ready.";
    m_cameraWorkerDone = true;
    m_tof_image_pos_y = -1.0f;
}

void ADIMainWindow::CameraPlay(int modeSelect, int viewSelect) {
    ImGuiWindowFlags
        overlayFlags = /*ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |*/
        ImGuiWindowFlags_NoResize | /*ImGuiWindowFlags_AlwaysAutoResize |*/
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    const bool imageIsHovered = ImGui::IsItemHovered();

    if (m_view_instance == nullptr) {
        return;
    }

    if (m_view_instance->m_ctrl->hasCamera()) {
        // Mode switch or starup
        if (m_mode_select_changed != modeSelect || m_capture_separate_enabled ||
            !m_is_playing) {
            if (m_mode_select_changed != modeSelect) {
                m_view_instance->m_ctrl->StopCapture();
            }

            PrepareCamera(modeSelect);
            OpenGLCleanUp();
            InitOpenGLABTexture();
            InitOpenGLDepthTexture();
            InitOpenGLPointCloudTexture();

            if (!m_off_line) {
                m_view_instance->m_ctrl->StartCapture(m_fps_expected);
                m_view_instance->m_ctrl->requestFrame();
            }
            else { // Offline: Always get the first frame
                if (m_offline_change_frame) {
                    m_view_instance->m_ctrl->requestFrame();
                    m_view_instance->m_ctrl->requestFrameOffline(m_off_line_frame_index);
                    m_offline_change_frame = false;
                }
            }
            m_capture_separate_enabled = false;
            m_mode_select_changed = modeSelect;

        } else if (m_view_selection_changed != viewSelect) {
            m_view_selection_changed = viewSelect;
            OpenGLCleanUp();
            InitOpenGLABTexture();
            InitOpenGLDepthTexture();
            InitOpenGLPointCloudTexture();
        }
    }

    std::shared_ptr<aditof::Frame> frame;
    if (synchronizeVideo(frame) >= 0) {
        if (frame != nullptr) {

            bool diverging = false;
            aditof::Metadata* metadata;
            aditof::Status status = frame->getData("metadata", (uint16_t**)&metadata);
            if (status == aditof::Status::OK && metadata != nullptr) {
				diverging = m_view_instance->m_ctrl->OutputDeltaTime(metadata->frameNumber);

				//LOG(INFO) << "Diverging: " << diverging;
            }

            bool haveAB = frame->haveDataType("ab");
            bool haveDepth = frame->haveDataType("depth");
            bool haveXYZ = frame->haveDataType("xyz");

            uint32_t numberAvailableDataTypes = 0;

            numberAvailableDataTypes += haveAB ? 1 : 0;
            numberAvailableDataTypes += haveDepth ? 1 : 0;
            numberAvailableDataTypes += haveXYZ ? 1 : 0;

            ImGuiIO& io = ImGui::GetIO();
            if (io.KeyShift) {
                
                if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
                    m_depth_line_values.clear();
                    m_depthLine.clear();
                    m_frame_window_position_state++;
                    if (m_frame_window_position_state > numberAvailableDataTypes)
                        m_frame_window_position_state = 0;
                }

                if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
                    m_depth_line_values.clear();
                    m_depthLine.clear();
                    m_frame_window_position_state--;
                    if (m_frame_window_position_state < 0)
                        m_frame_window_position_state = numberAvailableDataTypes;
                }
            }

            if (numberAvailableDataTypes == 3) {
                if (m_frame_window_position_state == 0) {
                    m_xyz_position = &m_dict_win_position["fr-main"];
                    m_ab_position = &m_dict_win_position["fr-sub1"];
                    m_depth_position = &m_dict_win_position["fr-sub2"];
                }
                else if (m_frame_window_position_state == 1) {
                    m_xyz_position = &m_dict_win_position["fr-sub2"];
                    m_ab_position = &m_dict_win_position["fr-main"];
                    m_depth_position = &m_dict_win_position["fr-sub1"];
                }
                else if (m_frame_window_position_state == 2) {
                    m_xyz_position = &m_dict_win_position["fr-sub1"];
                    m_ab_position = &m_dict_win_position["fr-sub2"];
                    m_depth_position = &m_dict_win_position["fr-main"];
                }
            } else if (numberAvailableDataTypes == 2) {

                if (!haveAB) {

					if (m_frame_window_position_state == 0) {
						m_xyz_position = &m_dict_win_position["fr-main"];
						m_depth_position = &m_dict_win_position["fr-sub1"];
					}
					else {
						m_xyz_position = &m_dict_win_position["fr-sub1"];
						m_depth_position = &m_dict_win_position["fr-main"];
					}
                } else if (!haveDepth) {

                    if (m_frame_window_position_state == 0) {
                        m_xyz_position = &m_dict_win_position["fr-main"];
                        m_ab_position = &m_dict_win_position["fr-sub1"];
                    }
                    else {
                        m_xyz_position = &m_dict_win_position["fr-sub1"];
                        m_ab_position = &m_dict_win_position["fr-main"];
                    }
                } else if (!haveXYZ) {

                    if (m_frame_window_position_state == 0) {
                        m_depth_position = &m_dict_win_position["fr-main"];
                        m_ab_position = &m_dict_win_position["fr-sub1"];
                    }
                    else {
                        m_depth_position = &m_dict_win_position["fr-sub1"];
                        m_ab_position = &m_dict_win_position["fr-main"];
                    }
                }
            } else {
                if (haveDepth) {
                    m_depth_position = &m_dict_win_position["fr-main"];
                } else if (haveAB) {
                    m_ab_position = &m_dict_win_position["fr-main"];
                } else if (haveXYZ) {
                    m_xyz_position = &m_dict_win_position["fr-main"];
                }
            }

            if (haveXYZ) {
                DisplayPointCloudWindow(overlayFlags);
            }
            if (haveAB) {
                DisplayActiveBrightnessWindow(overlayFlags);
            }
            if (haveDepth) {
                DisplayDepthWindow(overlayFlags);
            }
            DisplayInfoWindow(overlayFlags, diverging);
            DisplayControlWindow(overlayFlags, haveAB, haveDepth, haveXYZ);
            if (haveDepth) {
                DepthLinePlot(overlayFlags);
            }
        }
    }
}

void ADIMainWindow::CameraStop() {
    if (m_view_instance) {
        if (m_view_instance->m_ctrl) {
            OpenGLCleanUp();
            m_view_instance->m_ctrl->StopCapture();
            m_view_instance->m_ctrl->panicStop = false;
        }
    }
    /*if (initCameraWorker.joinable()) {
        initCameraWorker.join();
        m_cameraModes.clear();
        _cameraModes.clear();
    }*/
    m_focused_once = false;
    m_capture_separate_enabled = true;
    m_set_ab_win_position_once = true;
    m_set_depth_win_position_once = true;
    m_set_point_cloud_position_once = true;
    m_is_playing = false;
    m_fps_frame_received = 0;
    m_off_line_frame_index = 0;
}

void ADIMainWindow::RefreshDevices() {

    m_cameraWorkerDone = false;
    m_cameraModes.clear();
    _cameraModes.clear();
    if (initCameraWorker.joinable()) {
        initCameraWorker.join();
    }

    m_selected_device_index = -1;
    m_connected_devices.clear();
    m_configFiles.clear();
    m_cameras_list.clear();

    aditof::Status status;
    if (m_off_line) {
        status = m_system.getCameraList(m_cameras_list);
        for (size_t ix = 0; ix < m_cameras_list.size(); ++ix) {
            m_connected_devices.emplace_back(ix, "ToF Camera " +
                                                     std::to_string(ix));
        }
    } else {

        if (!m_skip_network_cameras) {
            // Add network camera
            m_system.getCameraList(m_cameras_list, m_cameraIp + m_ip_suffix);
            if (m_cameras_list.size() > 0) {
                uint32_t index =
                    static_cast<uint32_t>(m_connected_devices.size());
                m_connected_devices.emplace_back(
                    index, "ToF Camera" + std::to_string(index));
            }
        }
    }

    if (!m_connected_devices.empty()) {
        //Search for configuration files with .json extension
        m_config_selection = -1;
        fs::path _currPath = fs::current_path();
        std::vector<std::string> files;
        getFilesList(_currPath.string(), "*.json", files, false);

        for (size_t fileCnt = 0; fileCnt < files.size(); fileCnt++) {
            m_configFiles.emplace_back(fileCnt, files[fileCnt]);
        }

        if (!m_configFiles.empty() && m_config_selection == -1) {
            m_config_selection = 0;
        }
    }
}

void ADIMainWindow::HandleInterruptCallback() {
    aditof::SensorInterruptCallback cb = [this](aditof::Adsd3500Status status) {
        LOG(WARNING) << "status: " << status;
        ImGui::Begin("Interrupt");
        ImGui::Text("%i", status);
    };
    aditof::Status ret_status = aditof::Status::OK;
    auto camera = GetActiveCamera();
    if (!camera) {
        return;
    }
    ret_status = camera->getSensor()->adsd3500_register_interrupt_callback(cb);
    if (ret_status != aditof::Status::OK) {
        LOG(ERROR) << "Could not register interrupt callback";
        return;
    }
}