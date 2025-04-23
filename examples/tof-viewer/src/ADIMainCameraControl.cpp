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

extern uint8_t last_mode;
extern std::map<std::string, std::string> ini_params;
extern std::map<std::string, std::string> modified_ini_params;
extern std::map<std::string, std::string> last_ini_params;
extern bool use_modified_ini_params;

void ADIMainWindow::InitCamera() {
    if (view != NULL) { //Reset current imager
        LOG(INFO) << "Imager is reseting.";
        view.reset();
        LOG(INFO) << "Reset successful.";
    }

    std::string version = aditof::getApiVersion();
    LOG(INFO) << "Preparing camera. Please wait...\n";
    view = std::make_shared<adiviewer::ADIView>(
        std::make_shared<adicontroller::ADIController>(m_camerasList),
        "ToFViewer " + version);
    m_camerasList.clear();
    _cameraModes.clear();
    m_cameraModesDropDown.clear();
    m_cameraModesLookup.clear();

    aditof::Status status = aditof::Status::OK;
    auto camera = getActiveCamera(); //already initialized on constructor

    if (!camera) {
        LOG(ERROR) << "No cameras found!";
        return;
    }

    status = camera->initialize("");
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
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
            s = s + std::to_string(modeDetails.numberOfPhases) + " Phases";
        } else {
            s = s + "PCM";
        }
        m_cameraModesDropDown.emplace_back(modeDetails.modeNumber, s);
        m_cameraModesLookup[modeDetails.modeNumber] = s;
    }

    for (int i = 0; i < _cameraModes.size(); i++) {
        m_cameraModes.emplace_back(i, _cameraModes.at(i));
    }

    m_cameraWorkerDone = true;
}

void ADIMainWindow::PrepareCamera(uint8_t mode) {
    aditof::Status status = aditof::Status::OK;
    std::vector<aditof::FrameDetails> frameTypes;

    status = getActiveCamera()->setMode(mode);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return;
    }

    if (mode == last_mode) {
        if (!modified_ini_params.empty()) {
            if (use_modified_ini_params) {
                status = getActiveCamera()->setFrameProcessParams(
                    modified_ini_params);
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Could not set ini params";
                } else {
                    LOG(INFO) << "Using user defined ini parameters.";
                    use_modified_ini_params = false;
                    modified_ini_params.clear();
                    last_ini_params.clear();
                }
            }
        }
    } else {
        use_modified_ini_params = false;
        ini_params.clear();
        modified_ini_params.clear();
        last_ini_params.clear();
        last_mode = mode;
    }

    aditof::CameraDetails camDetails;
    status = getActiveCamera()->getDetails(camDetails);
    int totalCaptures = camDetails.frameType.totalCaptures;

    status = getActiveCamera()->adsd3500GetFrameRate(m_fps_expectedFPS);

    aditof::FrameDetails tmp = view->m_ctrl->m_recorder->getFrameDetails();
    tmp.totalCaptures = totalCaptures;
    view->m_ctrl->m_recorder->setFrameDetails(tmp);

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

    LOG(INFO) << "Camera ready.";
    m_cameraWorkerDone = true;
    tofImagePosY = -1.0f;
}

void ADIMainWindow::CameraPlay(int modeSelect, int viewSelect) {
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

            PrepareCamera(modeSelect);
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

    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
        m_frameWindowPosState++;
        if (m_frameWindowPosState > 2)
            m_frameWindowPosState = 0;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
        m_frameWindowPosState--;
        if (m_frameWindowPosState < 0)
            m_frameWindowPosState = 2;
    }

    if (m_frameWindowPosState == 1) {
        m_pcPosition = &dictWinPosition["fr-sub2"];
        m_abPosition = &dictWinPosition["fr-main"];
        m_depthPosition = &dictWinPosition["fr-sub1"];
    } else if (m_frameWindowPosState == 2) {
        m_pcPosition = &dictWinPosition["fr-sub1"];
        m_abPosition = &dictWinPosition["fr-sub2"];
        m_depthPosition = &dictWinPosition["fr-main"];
    } else {
        m_pcPosition = &dictWinPosition["fr-main"];
        m_abPosition = &dictWinPosition["fr-sub1"];
        m_depthPosition = &dictWinPosition["fr-sub2"];
    }

    displayDepth = true;
    displayAB = true;
    synchronizeVideo();
    DisplayPointCloudWindow(overlayFlags);
    DisplayActiveBrightnessWindow(overlayFlags);
    DisplayDepthWindow(overlayFlags);
    DisplayInfoWindow(overlayFlags);
    DisplayControlWindow(overlayFlags);
}

void ADIMainWindow::CameraStop() {
    m_focusedOnce = false;
    captureSeparateEnabled = true;
    captureBlendedEnabled = true;
    setABWinPositionOnce = true;
    setDepthWinPositionOnce = true;
    setPointCloudPositionOnce = true;
    setPointCloudPositionOnce = true;
    if (view) {
        if (view->m_ctrl) {
            view->m_ctrl->StopCapture();
            view->m_ctrl->panicStop = false;
        }
        view->cleanUp();
    }
    if (isRecording) {
        view->m_ctrl->stopRecording();
        isRecording = false;
    }
    openGLCleanUp();
    isPlaying = false;
    isPlayRecorded = false;
    m_fps_firstFrame = 0;
    m_fps_frameRecvd = 0;
}

void ADIMainWindow::RefreshDevices() {
    m_cameraWorkerDone = false;
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
        m_system.getCameraList(m_camerasList, m_cameraIp + m_ipSuffix);
        if (m_camerasList.size() > 0) {
            uint32_t index = static_cast<uint32_t>(m_connectedDevices.size());
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
        LOG(ERROR) << "Could not register interrupt callback";
        return;
    }
}