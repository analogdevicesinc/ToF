/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "camera_itof.h"
#include "calibration_itof.h"
#include "aditof/frame.h"
#include "aditof_internal.h"
#include "aditof/frame_operations.h"
#include "module_memory.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <glog/logging.h>
#include "cJSON/cJSON.h"
#include "cJSON/cJSON.c"
#include "tofi/floatTolin.h"
#include "tofi/tofi_config.h"
#include <iostream>


CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &tSensors)
    : m_depthSensor(depthSensor), m_devStarted(false), m_eepromInitialized(false),
      m_modechange_framedrop_count(0), m_xyzEnabled(false), m_xyzSetViaControl(false), m_loadedConfigData(false), m_tempFiles{},
      m_pulsatrixEnabled(false) {
    m_details.mode = "qmp";
    m_details.cameraId = "";

    // Define some of the controls of this camera
    m_controls.emplace("initialization_config", "");
    m_controls.emplace("powerUp", "call");
    m_controls.emplace("powerDown", "call");
    m_controls.emplace("syncMode", "0, 0");
    m_controls.emplace("loadModuleData", "call");
    m_controls.emplace("saveModuleCCB", "");
    m_controls.emplace("saveModuleCFG", "");

    m_noArgCallables.emplace("powerUp", std::bind(&CameraItof::powerUp, this));
    m_noArgCallables.emplace("powerDown",
                             std::bind(&CameraItof::powerUp, this));
    m_noArgCallables.emplace("loadModuleData",
                             std::bind(&CameraItof::loadModuleData, this));
    m_controls.emplace("enableDepthCompute", "on");
    m_controls.emplace("enableXYZframe", "off");

    // Check Depth Sensor
    if (!depthSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }

    aditof::SensorDetails sDetails;
    m_depthSensor->getDetails(sDetails);
    m_details.connection = sDetails.connectionType;

    std::string sensorName;
    m_depthSensor->getName(sensorName);
    if (sensorName == "pulsatrix") {
        m_pulsatrixEnabled = true;
    }

    // Look for EEPROM
    auto eeprom_iter =
        std::find_if(eeproms.begin(), eeproms.end(),
                [this](std::shared_ptr<aditof::StorageInterface> e) {
                        std::string name;
                        e->getName(name);
                        return name == m_eepromDeviceName;
                 });
    if (eeprom_iter == eeproms.end()) {
        if(!m_pulsatrixEnabled){
            LOG(WARNING) << "Could not find " << m_eepromDeviceName
                         << " while looking for storage";
        }
        m_eeprom = NULL;
    } else {
        m_eeprom = *eeprom_iter;
    }
}

CameraItof::~CameraItof() {
    cleanupTempFiles();
    freeConfigData();
    // m_device->toggleFsync();
    if (m_eepromInitialized) {
        m_eeprom->close();
    }
}

aditof::Status CameraItof::initialize() {
    using namespace aditof;

    LOG(INFO) << "Initializing camera";

    // Setting up the UVC filters, samplegrabber interface, Video renderer and filters
    // Setting UVC mediaformat and Running the stream is done once mode is set
    if (!m_devStarted) {
        Status status = m_depthSensor->open();
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to open device";
            return status;
        }
        m_devStarted = true;
    }

    m_depthSensor->getAvailableFrameTypes(m_availableSensorFrameTypes);

    // Parse config.json
    std::string config = m_controls["initialization_config"];
    std::ifstream ifs(config.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

    cJSON *config_json = cJSON_Parse(content.c_str());
    if (config_json != NULL) {
        // Get sensorfirmware file location
        const cJSON *json_sensorFirmware_file = cJSON_GetObjectItemCaseSensitive(config_json, "sensorFirmware");
        if (cJSON_IsString(json_sensorFirmware_file) && (json_sensorFirmware_file->valuestring != NULL)) {
            if (m_sensorFirmwareFile.empty()) {
                // save firmware file location
                m_sensorFirmwareFile = std::string(json_sensorFirmware_file->valuestring);
                LOG(INFO) << "Current sensor firmware is: "
                          << m_sensorFirmwareFile;
            } else {
                LOG(WARNING) << "Duplicate firmware file ignored: " << json_sensorFirmware_file->valuestring;
            }
        }

        // Get calibration file location
        const cJSON *json_ccb_calibration_file = cJSON_GetObjectItemCaseSensitive(config_json, "CCB_Calibration");
        if (cJSON_IsString(json_ccb_calibration_file) && (json_ccb_calibration_file->valuestring != NULL)) {
            if (m_ccb_calibrationFile.empty()) {
                // save calibration file location
                m_ccb_calibrationFile = std::string(json_ccb_calibration_file->valuestring);
                LOG(INFO) << "Current calibration file is: "
                          << m_ccb_calibrationFile;
            } else {
                LOG(WARNING) << "Duplicate calibration file ignored: " << json_ccb_calibration_file->valuestring;
            }
        }

        // Get optional eeprom type name
        const cJSON *eeprom_type_name = cJSON_GetObjectItemCaseSensitive(config_json, "MODULE_EEPROM_TYPE");
        if (cJSON_IsString(eeprom_type_name) && (eeprom_type_name->valuestring != NULL)) {
            m_eepromDeviceName = eeprom_type_name->valuestring;
        }

        // Get depth ini file location
        const cJSON *json_depth_ini_file = cJSON_GetObjectItemCaseSensitive(config_json, "DEPTH_INI");
        if (cJSON_IsString(json_depth_ini_file) && (json_depth_ini_file->valuestring != NULL))
            // save depth ini file location
            m_ini_depth = std::string(json_depth_ini_file->valuestring);
        LOG(INFO) << "Current Depth ini file is: " << m_ini_depth;

        // Get optional power config
        const cJSON *json_vaux_pwr = cJSON_GetObjectItemCaseSensitive(config_json, "VAUX_POWER_ENABLE");
        if (cJSON_IsString(json_vaux_pwr) && (json_vaux_pwr->valuestring != NULL)) {
            m_sensor_settings.push_back(std::make_pair(json_vaux_pwr->string, atoi(json_vaux_pwr->valuestring)));
        }

        // Get optional power config
        const cJSON *json_vaux_voltage = cJSON_GetObjectItemCaseSensitive(config_json, "VAUX_POWER_VOLTAGE");
        if (cJSON_IsString(json_vaux_voltage) && (json_vaux_voltage->valuestring != NULL)) {
            m_sensor_settings.push_back(std::make_pair(json_vaux_voltage->string, atoi(json_vaux_voltage->valuestring)));
        }
    } else if (!config.empty()) {
        LOG(ERROR) << "Couldn't parse config file: " << config.c_str();
        return Status::GENERIC_ERROR;
    }

    aditof::Status configStatus = loadConfigData();
    if (configStatus == aditof::Status::OK) {
        m_loadedConfigData = true;
    } else {
        LOG(ERROR) << "loadConfigData failed";
        return Status::GENERIC_ERROR;
    }

    m_depthSensor->getAvailableFrameTypes(m_availableSensorFrameTypes);

    if (m_eeprom) {
        void *handle;
        aditof::Status status = m_depthSensor->getHandle(&handle);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to obtain the handle";
            return status;
        }
        // Open communication with EEPROM
        status = m_eeprom->open(handle);
        if (status != Status::OK) {
            std::string name;
            m_eeprom->getName(name);
            LOG(ERROR) << "Failed to open EEPROM with name " << name;
        } else {
            m_eepromInitialized = true;
        }
    }

    LOG(INFO) << "Camera initialized";

    return Status::OK;
}

aditof::Status CameraItof::start() {
    using namespace aditof;
    Status status = Status::OK;

    if (m_pulsatrixEnabled) {
        m_CameraProgrammed = true;
        status = m_depthSensor->start();
        if (Status::OK != status) {
            LOG(ERROR) << "Error starting pulsatrix.";
            return Status::GENERIC_ERROR;
        }
        return aditof::Status::OK;
    }

    // Program the camera firmware only once, re-starting requires
    // only setmode and start the camera.
    uint16_t reg_address = 0x256;
    uint16_t reg_data;
    m_depthSensor->readRegisters(&reg_address, &reg_data, 1);

    if (reg_data) {
        m_CameraProgrammed = true;
        LOG(INFO) << "USEQ running. Skip CFG & CCB programming step\n";
    }

    if (!m_CameraProgrammed) {
        CalibrationItof calib(m_depthSensor);

        status = calib.writeConfiguration(m_sensorFirmwareFile);
        if (Status::OK != status) {
            LOG(ERROR) << "Error writing camera firmware.";
            return status;
        }

        status = calib.writeCalibration(m_ccb_calibrationFile);
        if (Status::OK != status) {
            LOG(ERROR) << "Error writing camera calibration data.";
            return status;
        }

        status = calib.writeSettings(m_sensor_settings);
        if (Status::OK != status) {
            LOG(ERROR) << "Error writing camera settings.";
            return status;
        }

       m_CameraProgrammed = true;
    }

    status = m_depthSensor->start();
    if (Status::OK != status) {
       LOG(ERROR) << "Error starting image sensor.";
       return Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status CameraItof::stop() {
    return m_depthSensor->stop();
}

aditof::Status CameraItof::setMode(const std::string &mode,
                                   const std::string &modeFilename = "") {
    LOG(INFO) << "Chosen mode: " << mode;
    m_details.mode = mode;

    return aditof::Status::OK;
}

aditof::Status
CameraItof::getAvailableModes(std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    availableModes = g_availableModes;

    return status;
}

aditof::Status CameraItof::setFrameType(const std::string& frameType) {
    using namespace aditof;
    Status status = Status::OK;

    auto frameTypeIt = std::find_if(
        m_availableSensorFrameTypes.begin(), m_availableSensorFrameTypes.end(),
        [&frameType](const DepthSensorFrameType& d) {
            return (d.type == frameType);
        });

    if (frameTypeIt == m_availableSensorFrameTypes.end()) {
        LOG(WARNING) << "Frame type: " << frameType
            << " not supported by camera";
        return Status::INVALID_ARGUMENT;
    }

    setMode(frameType);

    status = m_depthSensor->setFrameType(*frameTypeIt);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set frame type";
        return status;
    }

    if ((frameType == "pcm")) {
        m_controls["enableDepthCompute"] = "off";
    }

    // Store the frame details in camera details
    m_details.frameType.type = (*frameTypeIt).type;
    // TO DO: m_details.frameType.cameraMode =
    m_details.frameType.width = ModeInfo::getInstance()->getModeInfo(frameType).width;
    m_details.frameType.height = ModeInfo::getInstance()->getModeInfo(frameType).height;
    if(!m_pulsatrixEnabled){
        m_details.frameType.totalCaptures = ModeInfo::getInstance()->getModeInfo(frameType).subframes;
    } else {
        m_details.frameType.totalCaptures = 1;
    }
    m_details.frameType.dataDetails.clear();
    for (const auto item : (*frameTypeIt).content) {
        if (item.type == "xyz" && !m_xyzEnabled) {
            continue;
        }

        FrameDataDetails fDataDetails;
        fDataDetails.type = item.type;
        fDataDetails.width = item.width;
        fDataDetails.height = item.height;
        fDataDetails.subelementSize = sizeof(uint16_t);
        fDataDetails.subelementsPerElement = 1;

        if (item.type == "xyz"){
            fDataDetails.subelementsPerElement = 3;
        }

        m_details.frameType.dataDetails.emplace_back(fDataDetails);
    }

    if (m_controls["enableDepthCompute"] == "on" && ((m_details.frameType.totalCaptures > 1) || m_pulsatrixEnabled)){
        status = initComputeLibrary();
        if (Status::OK != status) {
            LOG(ERROR) << "Initializing compute libraries failed.";
            return Status::GENERIC_ERROR;
        }
    }
    else {
        freeComputeLibrary();
    }

    return status;
}

aditof::Status CameraItof::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    using namespace aditof;
    Status status = Status::OK;
    availableFrameTypes.clear();

    for (const auto &frameType : m_availableSensorFrameTypes) {
        availableFrameTypes.emplace_back(frameType.type);
    }

    return status;
}

aditof::Status setAttributesByMode(aditof::Frame& frame, const ModeInfo::modeInfo& modeInfo){
    aditof::Status status = aditof::Status::OK;

    frame.setAttribute("embed_hdr_length", std::to_string(EMBED_HDR_LENGTH));
    frame.setAttribute("mode", std::to_string(modeInfo.mode));
    frame.setAttribute("width", std::to_string(modeInfo.width));
    frame.setAttribute("height", std::to_string(modeInfo.height));
    frame.setAttribute("total_captures", std::to_string(modeInfo.subframes));
    frame.setAttribute("embed_width", std::to_string(modeInfo.embed_width));
    frame.setAttribute("embed_height", std::to_string(modeInfo.embed_height));
    frame.setAttribute("passive_ir", std::to_string(modeInfo.passive_ir));

    return status;
}

aditof::Status CameraItof::requestFrame(aditof::Frame *frame,
                                        aditof::FrameUpdateCallback /*cb*/) {
    using namespace aditof;
    Status status = Status::OK;
    std::string totalCapturesStr;
    uint8_t totalCaptures;
     ModeInfo::modeInfo aModeInfo;

    if (frame == nullptr){
        return Status::INVALID_ARGUMENT;
    }

    status = getCurrentModeInfo(aModeInfo);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get mode info";
        return status;
    }

    setAttributesByMode(*frame, aModeInfo);

    FrameDetails frameDetails;
    frame->getDetails(frameDetails);

    if (m_details.frameType != frameDetails) {
        frame->setDetails(m_details.frameType);
    }

    if(!m_pulsatrixEnabled){
        frame->getAttribute("total_captures", totalCapturesStr);
        totalCaptures = std::atoi(totalCapturesStr.c_str());
    } else {
        totalCaptures = 1;
    }

    uint16_t *frameDataLocation = nullptr;
    if ((m_details.frameType.type == "pcm")){
        frame->getData("ir", &frameDataLocation);
    } else if (m_details.frameType.type == "") {
        LOG(ERROR) << "Frame type not found!";
            return Status::INVALID_ARGUMENT;
    }
    else {
        frame->getData("raw", &frameDataLocation);
    }

    if(!m_pulsatrixEnabled){
        uint16_t *embedFrame = nullptr;
        frame->getData("frameData", &embedFrame);

        status = m_depthSensor->getFrame(embedFrame);

        if (status != Status::OK) {
            LOG(WARNING) << "Failed to get embedded frame from device";
            return status;
        }

        for (unsigned int i = 0; i < (m_details.frameType.height * m_details.frameType.width * totalCaptures); ++i) {
            frameDataLocation[i] = frameDataLocation[i] >> 4;
            frameDataLocation[i] = Convert11bitFloat2LinearVal(frameDataLocation[i]);
        }

    } else {
        uint16_t *depthData = nullptr;
        frame->getData("depth", &depthData);
        status = m_depthSensor->getFrame(depthData);
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to get frame from pulsatrix";
            return status;
        }
        for (unsigned int i = 0; i < (m_details.frameType.height * m_details.frameType.width); ++i) {
            depthData[i] = depthData[i] >> 4;
            // TO DO: check if we need this conversion for pulsatrix
            //depthData[i] = Convert11bitFloat2LinearVal(depthData[i]);
        }

        uint32_t ret = TofiCompute(depthData, m_tofi_compute_context, NULL);

        if (ret != ADI_TOFI_SUCCESS) {
            LOG(INFO) << "TofiCompute failed for pulsatrix";
            return Status::GENERIC_ERROR;
        }

        memcpy(depthData, (uint8_t *)m_tofi_compute_context->p_depth_frame, 
            (m_details.frameType.height * m_details.frameType.width * sizeof(uint16_t)));

        if (m_xyzEnabled) {
            uint16_t *xyzData = nullptr;
            frame->getData("xyz", &depthData);
            memcpy(xyzData, m_tofi_compute_context->p_xyz_frame, 
                (m_details.frameType.height * m_details.frameType.width * sizeof(aditof::Point3I)));
        }
    }
    
    uint16_t *header = nullptr;
    if(!m_pulsatrixEnabled){
        frame->getData("header", &header);
    }
    
    if (!frameDataLocation /*&& !header*/) {
        LOG(WARNING) << "getframe failed to allocated valid frame";
        return status;
    }

    uint16_t embed_width = 0;
    uint16_t embed_height = 0;

    embed_height = aModeInfo.embed_height;
    embed_width = aModeInfo.embed_width;

    if((totalCaptures > 1) && (m_controls["enableDepthCompute"] == "on") && !m_pulsatrixEnabled) {
 
        if (NULL == m_tofi_compute_context) {
            LOG(ERROR) << "Depth compute libray not initialized";
            return Status::GENERIC_ERROR;
        }
        uint32_t ret = TofiCompute(frameDataLocation, m_tofi_compute_context, NULL);

        if (ret != ADI_TOFI_SUCCESS) {
            LOG(ERROR) << "TofiCompute failed";
            return Status::GENERIC_ERROR;
        }

        uint16_t *depthFrameLocation;
        frame->getData("depth", &depthFrameLocation);
        memcpy(depthFrameLocation, (uint8_t *)m_tofi_compute_context->p_depth_frame,
            (m_details.frameType.height * m_details.frameType.width * sizeof(uint16_t)));

        uint16_t *irFrameLocation;
        frame->getData("ir", &irFrameLocation);
        memcpy(irFrameLocation, m_tofi_compute_context->p_ab_frame, (m_details.frameType.height * m_details.frameType.width * sizeof(uint16_t)));

        applyCalibrationToFrame(frameDataLocation, std::atoi(m_details.mode.c_str()));

        if (m_xyzEnabled) {
            uint16_t* xyzFrameLocation;
            frame->getData("xyz", &xyzFrameLocation);
            memcpy(xyzFrameLocation, m_tofi_compute_context->p_xyz_frame, (m_details.frameType.height * m_details.frameType.width * sizeof(aditof::Point3I)));
        }
    }

    return Status::OK;
}

aditof::Status CameraItof::getDetails(aditof::CameraDetails &details) const {
    using namespace aditof;
    Status status = Status::OK;

    details = m_details;

    return status;
}

std::shared_ptr<aditof::DepthSensorInterface> CameraItof::getSensor() {
    return m_depthSensor;
}

aditof::Status CameraItof::getEeproms(
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms) {
    eeproms.clear();
    eeproms.emplace_back(m_eeprom);

    return aditof::Status::OK;
}

aditof::Status CameraItof::getTemperatureSensors(
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &sensors) {
    sensors.clear();
    sensors.emplace_back(m_tempSensor);

    return aditof::Status::OK;
}

aditof::Status
CameraItof::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls.clear();
    controls.reserve(m_controls.size());
    for (const auto &item : m_controls) {
        controls.emplace_back(item.first);
    }

    return status;
}

aditof::Status CameraItof::setControl(const std::string &control,
                                      const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) > 0) {
        if (value == "call") {
            return m_noArgCallables.at(control)();
        } else if (control == "syncMode") {
            // TO DO: parse value and get the two parameters (mode, level)
            uint8_t mode = 0;
            uint8_t level = 0;
            return setCameraSyncMode(mode, level);
        } else if (control == "saveModuleCCB") {
            return saveCCBToFile(value);
        } else if (control == "saveModuleCFG"){
            return saveCFGToFile(value);
        } else if (control == "enableXYZframe") {
            if (value == "on")
                return enableXYZframe(true);
            else if (value == "off")
                return enableXYZframe(false);
            else
                return Status::INVALID_ARGUMENT;
        } else {
            m_controls[control] = value;
        }
    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

aditof::Status CameraItof::getControl(const std::string &control,
                                      std::string &value) const {
    using namespace aditof;
    Status status = Status::OK;

    if (m_controls.count(control) > 0) {
        value = m_controls.at(control);
    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return status;
}

aditof::Status CameraItof::initComputeLibrary(void) {
    aditof::Status status = aditof::Status::OK;

    LOG(INFO) << "initComputeLibrary";
    //freeComputeLibrary();
    uint8_t convertedMode;

    size_t jsonFileSize = m_jsonFileSize;

    status = convertCameraMode(m_details.mode, convertedMode);

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return aditof::Status::GENERIC_ERROR;
    }

    if (m_loadedConfigData) {
        ConfigFileData calData = {m_calData.p_data, m_calData.size};
        uint32_t status = ADI_TOFI_SUCCESS;

        if (!m_ini_depth.empty()) {
            uint8_t *tempDataParser = new uint8_t[m_depthINIData.size];
            memcpy(tempDataParser, m_depthINIData.p_data, m_depthINIData.size);
            ConfigFileData depth_ini = {tempDataParser, m_depthINIData.size};

            if(m_pulsatrixEnabled){
                status = GetXYZ_DealiasData((ConfigFileData *)&m_calData, m_xyz_dealias_data);
                if (status != ADI_TOFI_SUCCESS) {
                    LOG(ERROR) << "Failed to GetCalibrationData";
                    return aditof::Status::INVALID_ARGUMENT;
                }

                m_tofi_config = InitTofiConfig_isp((ConfigFileData *)&depth_ini,
                                                   convertedMode, &status,
                                                   m_xyz_dealias_data);

            } else {
                m_tofi_config = InitTofiConfig(&calData, NULL, &depth_ini, convertedMode, &status);
            }

            delete[] tempDataParser;

        } else {
            m_tofi_config = InitTofiConfig(&calData, NULL, NULL, convertedMode, &status);
        }

        if ((m_tofi_config == NULL) || (m_tofi_config->p_tofi_cal_config == NULL) || (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed";
            return aditof::Status::GENERIC_ERROR;

        } else {
            m_tofi_compute_context = InitTofiCompute(m_tofi_config->p_tofi_cal_config, &status);
            if (m_tofi_compute_context == NULL || status != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "InitTofiCompute failed";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    } else {
        LOG(ERROR) << "Could not initialize compute library because config data hasn't been loaded";
        return aditof::Status::GENERIC_ERROR;
    }

    if (status != aditof::Status::OK) {
        freeComputeLibrary();
    }
    return status;
}

aditof::Status CameraItof::freeComputeLibrary(void) {

    freeConfigData();

    if (NULL != m_tofi_compute_context) {
        LOG(INFO) << "freeComputeLibrary";        
        FreeTofiCompute(m_tofi_compute_context);
        m_tofi_compute_context = NULL;
    }

    if (m_tofi_config != NULL) {
        FreeTofiConfig(m_tofi_config);
        m_tofi_config = NULL;
    }
    return aditof::Status::OK;
}

aditof::Status CameraItof::loadConfigData(void) {
    uint32_t status = 0;
    uint32_t calFileSize = 0;
    uint32_t jsonFileSize = 0;
    uint32_t iniFileSize = 0;
    freeConfigData();

    aditof::Status retErr = aditof::Status::GENERIC_ERROR;

    if (!m_ini_depth.empty()) {
        m_depthINIData = LoadFileContents(const_cast<char *>(m_ini_depth.c_str()));
    }

    if (!m_ccb_calibrationFile.empty()) {
        m_calData = LoadFileContents(const_cast<char*>(m_ccb_calibrationFile.c_str()));
    }

    // XYZ set through camera control takes precedence over the setting from .ini file
    if (!m_xyzSetViaControl) {
        std::string depthData((char *)m_depthINIData.p_data,
                              GetDataFileSize(const_cast<char *>(m_ini_depth.c_str())));
        size_t pos = depthData.find("xyzEnable", 0);

        if (pos != std::string::npos) {
            if (depthData.at(pos + strlen("xyzEnable=")) == '1') {
                m_xyzEnabled = true;
            }
        }
    }

    m_jsonFileSize = jsonFileSize;

    return aditof::Status::OK;
}

void CameraItof::freeConfigData(void)
{
    // TO DO:
}

aditof::Status CameraItof::isValidFrame(const int numTotalFrames) {
    using namespace aditof;

    ModeInfo::modeInfo aModeInfo;
    if (Status::OK != getCurrentModeInfo(aModeInfo)) {
        return Status::GENERIC_ERROR;
    }

    if (aModeInfo.subframes == numTotalFrames) {
        return (aditof::Status::OK);
    }

    return (aditof::Status::GENERIC_ERROR);
}

aditof::Status CameraItof::isValidMode(const uint8_t /*hdr_mode*/) {
  /*  using namespace aditof;
    unsigned int mode = 0;
    m_depthSensor->getMode(mode);

    if ((static_cast<uint8_t>(mode)) == (hdr_mode)) {
        return (aditof::Status::OK);
    }

    return (aditof::Status::GENERIC_ERROR);*/
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
CameraItof::processFrame(uint8_t *rawFrame, uint16_t *captureData,
                         uint8_t *head, const uint16_t embed_height,
                         const uint16_t embed_width,
                         aditof::Frame *frame) {
    using namespace aditof;
    Status status = Status::OK;

    // Read header data and process image
    uint16_t REG_CAPTURE_ID = 0;
    uint16_t FrameWidth = 0;
    uint16_t FrameHeight = 0;
    uint16_t FrameNum = 0;
    uint8_t totalCaptures = 0;
    uint8_t captureID = 0;
    uint16_t chipID = 0;
    uint16_t REG_MODE_ID_CURR = 0;
    uint8_t Mode = 0;

    /*chipID = (rawFrame[0] | (rawFrame[1] << 8)) >> 4; // header[0]: [15:0] Chip ID register
    if (chipID != CHIPID) {
        LOG(WARNING) << "Invalid ChipID";
        //return Status::GENERIC_ERROR;
    }

    // REG_CAPTURE_ID[49] word:contains [5:0] = Capture Number; [11:6] = Number of Captures per Frame; [15:12] = Number of Frequencies Per Frame
    REG_CAPTURE_ID = rawFrame[REG_CAPTURE_ID_LOC * 2] | (rawFrame[REG_CAPTURE_ID_LOC * 2 + 1] << 8);
    totalCaptures = (REG_CAPTURE_ID >> 6) & 0x3F;
    captureID = REG_CAPTURE_ID & 0x3F;

    // REG_MODE_ID_CURR[] word:contains  [2:0] = Mode;   [15:0] = uSeq0
    REG_MODE_ID_CURR = rawFrame[REG_MODE_ID_CURR_LOC * 2] | (rawFrame[REG_MODE_ID_CURR_LOC * 2 + 1] << 8);
    Mode = REG_MODE_ID_CURR & 0xF;

    // If mode feild from the header is not as mode set by user, drop the frame
    if (isValidMode(Mode) != Status::OK) {
        m_modechange_framedrop_count++;
        return Status::BUSY; // This frame is from the previous mode drop it
    }

    if (m_modechange_framedrop_count >= 1000) {
        return Status::GENERIC_ERROR; // Camera is still in previous mode, never switched to new mode after 1000 frames
    }

    if (isValidFrame(totalCaptures + 1) != Status::OK) {
        return Status::GENERIC_ERROR; // Invalid number of subframes
    }*/

    //FrameWidth = rawFrame[FRAME_WIDTH_LOC] | (rawFrame[FRAME_WIDTH_LOC + 1] << 8);    // header[6]: [10:0] The width of the ROI in pixels
    //FrameHeight = rawFrame[FRAME_HEIGHT_LOC] | (rawFrame[FRAME_HEIGHT_LOC + 1] << 8); // header[7]: [10:0] The height of the ROI in pixels
    //FrameNum = rawFrame[FRAME_NUM_LOC] | (rawFrame[FRAME_NUM_LOC + 1] << 8);          // header[4]: [15:0] FrameNumber running count
    std::string fn;
    frame->getAttribute("total_captures", fn);
    FrameNum = std::atoi(fn.c_str());
    FrameWidth = 1024; //TODO-hardcoded
    FrameHeight = 1024;
    //frame->setAttribute("frameNum", std::to_string(FrameNum));

    if (captureID != 0) {
        // Todo: what are the valid captureID values? handle accordingly
    }

    // Ex: for Mode=5, rawSubFrameSize will (12289 * 640 * 2)/(9 + 1) - 128
    uint64_t rawSubFrameSize = ((embed_height * embed_width * 2) / (totalCaptures + 1)) - 128;
    uint64_t subFrameSize = FrameWidth * FrameHeight; // capture size without header
    uint16_t *p = (uint16_t*) rawFrame;

    for (int i = 0; i < FrameNum * FrameWidth * FrameHeight; ++ i) {
        captureData[i] = p[i] >> 4;
    }

    // parse embedded frame and get header data
    /*for (int fr = 0; fr < 1 + totalCaptures; fr++) {
        uint64_t subFrameOffset = fr * (EMBED_HDR_LENGTH + rawSubFrameSize);
        uint8_t *ptr = rawFrame + subFrameOffset;

        REG_CAPTURE_ID = rawFrame[REG_CAPTURE_ID_LOC * 2 + subFrameOffset] |
                         (rawFrame[REG_CAPTURE_ID_LOC * 2 + 1 + subFrameOffset] << 8);

        uint16_t newFrameNum = rawFrame[FRAME_NUM_LOC + subFrameOffset] |
                               (rawFrame[(FRAME_NUM_LOC + 1) + subFrameOffset] << 8);

        //captureID = REG_CAPTURE_ID & 0x3F;

        //memcpy(&head[fr * EMBED_HDR_LENGTH], ptr, sizeof(uint8_t) * EMBED_HDR_LENGTH);

        //chipID =
        //    rawFrame[CHIPID_LOC + subFrameOffset] | (rawFrame[(CHIPID_LOC + 1) + subFrameOffset] << 8);

        if (chipID != CHIPID) {
            LOG(WARNING) << "Invalid frame: invalid chipID";
            // return Status::GENERIC_ERROR;    //Todo: should we return from here?
        }

        if (captureID != fr) {
            LOG(WARNING) << "Invalid frame: invalid captureID";
            // return Status::GENERIC_ERROR;    //Todo: should we return from here?
        }

        if (newFrameNum != FrameNum) {
            VLOG(1) << "Invalid frame: invalid newFrameNum=" << newFrameNum << " FrameNum=" << FrameNum << " capture=" << fr
                     << " FrameWidth=" << FrameWidth << " FrameHeight=" << FrameHeight;
            // return Status::GENERIC_ERROR;   //Todo: should we return from here?
        }

        int k = (fr + 1) * EMBED_HDR_LENGTH + fr * rawSubFrameSize;
        // 12->16 bit conversion and store the 16bit data in raw frame buffer provided
        for (uint64_t i = 0; i < rawSubFrameSize / 3; i++) {

            captureData[2 * i + fr * subFrameSize] = ((int16_t)rawFrame[k + 3 * i]) | ((((int16_t)rawFrame[k + 3 * i + 1]) & 0xF) << 8);
            captureData[2 * i + 1 + fr * subFrameSize] = (((int16_t)rawFrame[k + 3 * i + 1]) >> 4) | (((int16_t)rawFrame[k + 3 * i + 2]) << 4);
        }

        for (uint64_t i = 0; i < rawSubFrameSize; ++ i) {
            //captureData[i] = (((rawFrame[i] & 0xF) << 8) >> 4) | (rawFrame[i] >> 8);
            captureData[i] = rawFrame[i] >> 4;
        }

        for (int i = 0; i < rawSubFrameSize; ++ i) {
            captureData[i] = rawFrame[i];
        }
    }*/

    return status;
}

aditof::Status CameraItof::getCurrentModeInfo(ModeInfo::modeInfo &info) {
    using namespace aditof;
    Status status = Status::OK;
    uint8_t convertedMode;

    status = convertCameraMode(m_details.mode, convertedMode);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return aditof::Status::GENERIC_ERROR;
    }

    ModeInfo *pModeInfo = ModeInfo::getInstance();
    if (pModeInfo) {
        info = pModeInfo->getModeInfo(convertedMode);
        return Status::OK;
    }
    return Status::GENERIC_ERROR;
}

aditof::Status CameraItof::cleanupTempFiles() {
    using namespace aditof;
    Status status = Status::OK;

    std::string filename = m_tempFiles.jsonFile;
    if (!filename.empty()) {
        if( std::remove(filename.c_str()) != 0 ) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    filename = m_tempFiles.ccbFile;
    if (!filename.empty()) {
        if( std::remove(filename.c_str()) != 0 ) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    filename = m_tempFiles.cfgFile;
    if (!filename.empty()) {
        if( std::remove(filename.c_str()) != 0 ) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    m_tempFiles = {};

    return status;
}

aditof::Status CameraItof::powerUp() {
    // TO DO: decide if we need to keep this method
    return aditof::Status::OK;
}

aditof::Status CameraItof::powerDown() {
    // TO DO: decide if we need to keep this method
    return aditof::Status::OK;
}

aditof::Status CameraItof::setCameraSyncMode(uint8_t mode, uint8_t level) {
    //defined in device_interface.h -> depth_sensor_interface.h
    //return m_depthSensor->setCameraSyncMode(mode, level);
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::loadModuleData() {
    using namespace aditof;
    Status status = Status::OK;

    if (m_details.connection == aditof::ConnectionType::OFFLINE) {
        return status;
    }

    if(m_pulsatrixEnabled) {
        return initialize();
    }
    
    cleanupTempFiles();
    if (!m_eepromInitialized) {
        LOG(ERROR) << "Memory interface can't be accessed";
        return Status::GENERIC_ERROR;
    }
    std::string tempJsonFile;

    ModuleMemory flashLoader(m_eeprom);
    flashLoader.readModuleData(tempJsonFile, m_tempFiles);

    std::string serialNumber;
    flashLoader.getSerialNumber(serialNumber);
    int prefixCount = 2;// I'm asuming the first 2 characters are "D:" which we don't need
    std::string shortName = serialNumber.substr(prefixCount, serialNumber.find(" ") - prefixCount);
    m_details.cameraId = shortName;

    // m_depthSensor->cameraReset(); TO DO: figure out if this is required or how to do the reset since there is currenlty no cameraReset() in DepthSensorInterface

    if (!tempJsonFile.empty()) {
        setControl("initialization_config", tempJsonFile);
        return initialize();
    } else {
        LOG(ERROR) << "Error loading module data";
        return Status::GENERIC_ERROR;
    }
}

aditof::Status CameraItof::applyCalibrationToFrame(uint16_t *frame, const unsigned int mode) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::saveCCBToFile(const std::string &filePath) const {
    if (filePath.empty()) {
        LOG(ERROR) << "File path where CCB should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (m_tempFiles.ccbFile.empty()) {
        LOG(ERROR) << "CCB files is unavailable. Perhaps CCB content was not read from module.";
        return aditof::Status::UNAVAILABLE;
    }

    std::ifstream source(m_tempFiles.ccbFile.c_str(), std::ios::binary);
    std::ofstream destination(filePath, std::ios::binary);
    destination << source.rdbuf();

    return aditof::Status::OK;
}

aditof::Status CameraItof::saveCFGToFile(const std::string &filePath) const {
    if (filePath.empty()) {
        LOG(ERROR) << "File path where CFG should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (m_tempFiles.cfgFile.empty()) {
        LOG(ERROR) << "CFG files is unavailable. Perhaps CFG content was not "
                      "read from module.";
        return aditof::Status::UNAVAILABLE;
    }

    std::ifstream source(m_tempFiles.cfgFile.c_str(), std::ios::binary);
    std::ofstream destination(filePath, std::ios::binary);
    destination << source.rdbuf();

    return aditof::Status::OK;
}

aditof::Status CameraItof::enableXYZframe(bool en) {
    m_xyzEnabled = en;
    m_xyzSetViaControl = true;

    return aditof::Status::OK;
}
