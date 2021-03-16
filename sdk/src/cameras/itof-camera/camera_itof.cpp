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

#include <algorithm>
#include <array>
#include <glog/logging.h>
#include "cJSON/cJSON.h"
#include "cJSON/cJSON.c"
#include "tofi/floatTolin.h"
#include "tofi/tofi_utils.h"

CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &tSensors)
    : m_depthSensor(depthSensor), m_devStarted(false),
      m_modechange_framedrop_count(0) {
    m_details.mode = "short_throw";
    m_details.cameraId = "";

    // Define some of the controls of this camera
    m_controls.emplace("initialization_config", "");
    m_controls.emplace("powerUp", "call");
    m_controls.emplace("powerDown", "call");
    m_controls.emplace("syncMode", "0, 0");
    m_controls.emplace("loadModuleData", "call");

    m_noArgCallables.emplace("powerUp", std::bind(&CameraItof::powerUp, *this));
    m_noArgCallables.emplace("powerDown",
                             std::bind(&CameraItof::powerUp, *this));
    m_noArgCallables.emplace("loadModuleData",
                             std::bind(&CameraItof::loadModuleData, *this));

    // Check Depth Sensor
    if (!depthSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }
    aditof::SensorDetails sDetails;
    m_depthSensor->getDetails(sDetails);
    m_details.connection = sDetails.connectionType;

}

CameraItof::~CameraItof() {
    cleanupTempFiles();
    freeConfigData();
    // m_device->toggleFsync();
}

aditof::Status CameraItof::initialize() { return aditof::Status::OK; }
//For now we keep the device open all the time
aditof::Status CameraItof::start() { return aditof::Status::OK; }

aditof::Status CameraItof::stop() { return aditof::Status::OK; }

aditof::Status CameraItof::setMode(const std::string &mode,
                                   const std::string &modeFilename) {
    return aditof::Status::OK;
}

aditof::Status
CameraItof::getAvailableModes(std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;

    // Dummy data. To remove when implementig this method
    availableModes.emplace_back("short_throw");
    availableModes.emplace_back("long_throw");
    availableModes.emplace_back("aHat1"); 
    availableModes.emplace_back("pcm");
    availableModes.emplace_back("long_throw_native");
    availableModes.emplace_back("mp_pcm");
    availableModes.emplace_back("chip_char");
    availableModes.emplace_back("qmp");
    availableModes.emplace_back("pcm8");
    availableModes.emplace_back("ahat2");
    availableModes.emplace_back("mp");

    return status;
}

aditof::Status CameraItof::setFrameType(const std::string &frameType) {
    return aditof::Status::OK;
}

aditof::Status CameraItof::getAvailableFrameTypes(
    std::vector<std::string> &availableFrameTypes) const {
    return aditof::Status::OK;
}

aditof::Status CameraItof::requestFrame(aditof::Frame *frame,
                                        aditof::FrameUpdateCallback /*cb*/) {
    return aditof::Status::OK;
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

    return aditof::Status::OK;
}

aditof::Status CameraItof::getTemperatureSensors(
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &sensors) {
    sensors.clear();

    return aditof::Status::OK;
}

aditof::Status
CameraItof::getAvailableControls(std::vector<std::string> &controls) const {
    using namespace aditof;
    Status status = Status::OK;

    controls.empty();
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

aditof::Status CameraItof::convertCameraMode(const std::string &modes, uint8_t *convertedMode) {
    std::vector<std::string> availableModes;
    aditof::Status status = aditof::Status::OK;
    if (status != getAvailableModes(availableModes)){
        return aditof::Status::GENERIC_ERROR;
    };

    auto it = std::find (availableModes.begin(), availableModes.end(), modes);
    if (it == availableModes.end()){
        return aditof::Status::GENERIC_ERROR;
    }

    *convertedMode = (it - availableModes.begin());
    return status;
}

aditof::Status CameraItof::initComputeLibrary(void) {
   aditof::Status status = aditof::Status::OK;

    LOG(INFO) << "initComputeLibrary";
    freeComputeLibrary();
    uint8_t convertedMode;

    aditof::Status configStatus;
    size_t calFileSize = 0, jsonFileSize = 0, iniFileSize = 0;
    std::tie(configStatus, calFileSize, jsonFileSize, iniFileSize) = loadConfigData();

    status = convertCameraMode(m_details.mode, &convertedMode);

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return aditof::Status::GENERIC_ERROR;
    }

    if (configStatus == aditof::Status::OK) {
        ConfigFileData calData = {m_calData, calFileSize};
        uint32_t status = ADI_TOFI_SUCCESS;

        if (!m_ini_depth.empty()) {

            ConfigFileData depth_ini = {m_depthINIData, iniFileSize};
            
            m_tofi_config = InitTofiConfig(&calData, NULL, &depth_ini, convertedMode, &status);
        } else {
            m_tofi_config = InitTofiConfig(&calData, NULL, NULL, convertedMode, &status);
        }

        if ((m_tofi_config == NULL) || (m_tofi_config->p_tofi_cal_config == NULL) || (status != ADI_TOFI_SUCCESS)) {
            LOG(INFO) << "InitTofiConfig failed";
            return aditof::Status::GENERIC_ERROR;

        } else {
            m_tofi_compute_context = InitTofiCompute(m_tofi_config->p_tofi_cal_config, &status);
            if (m_tofi_compute_context == NULL || status != ADI_TOFI_SUCCESS) {
                LOG(INFO) << "InitTofiCompute failed";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    } else {
        LOG(INFO) << "loadConfigData failed";
        return aditof::Status::GENERIC_ERROR;
    }

    if (status != aditof::Status::OK) {
        freeComputeLibrary();
    }
    return status;
}

aditof::Status CameraItof::freeComputeLibrary(void) {
       LOG(INFO) << "freeComputeLibrary";

    freeConfigData();

    if (NULL != m_tofi_compute_context) {
        FreeTofiCompute(m_tofi_compute_context);
        m_tofi_compute_context = NULL;
    }

    if (m_tofi_config != NULL) {
        FreeTofiConfig(m_tofi_config);
        m_tofi_config = NULL;
    }
    return aditof::Status::OK;
}

std::tuple<aditof::Status, int, int, int> CameraItof::loadConfigData(void) {
       uint32_t calFileSize = 0, jsonFileSize = 0, iniFileSize = 0, status = 0;
    freeConfigData();

    std::tuple<aditof::Status, int, int, int> retErr = std::make_tuple(aditof::Status::GENERIC_ERROR, 0, 0, 0);

#if 0 // Jason file are hardcoded un-comment for next release
      // 
    jsonFileSize = GetDataFileSize(JASON_CONFIG_FILE);
    m_jconfigData = new uint8_t[jsonFileSize];

    if (m_jconfigData == NULL) {
        return retErr;
    }
    status &= LoadFileContents(JASON_CONFIG_FILE, m_jconfigData,
        &jsonFileSize);
    if (status == 0) {
        LOG(WARNING) << "Unable to load jfile contents\n";
    }

#endif

    if (!m_ini_depth.empty()) {

        iniFileSize = GetDataFileSize(m_ini_depth.c_str());
        m_depthINIData = new uint8_t[iniFileSize];
        if (m_depthINIData == NULL) {
            return retErr;
        }

        status = LoadFileContents(m_ini_depth.c_str(), m_depthINIData, &iniFileSize);
        if (status == 0) {
            LOG(WARNING) << "Unable to load depth ini contents\n";
            return retErr;
        }
    }

    calFileSize = GetDataFileSize(m_ccb_calibrationFile.c_str());
    m_calData = new uint8_t[calFileSize];
    if (m_calData == NULL) {
        return retErr;
    }
    status = LoadFileContents(m_ccb_calibrationFile.c_str(), m_calData, &calFileSize);
    if (status == 0) {
        LOG(INFO) << "Unable to load cfile contents\n";
        return retErr;
    }

    return std::make_tuple(aditof::Status::OK, calFileSize, jsonFileSize, iniFileSize);
//    return std::make_tuple(aditof::Status::UNAVAILABLE, 0, 0, 0);
}

void CameraItof::freeConfigData(void) {
    
    delete (m_jconfigData);
    m_jconfigData = NULL;

    delete (m_depthINIData);
    m_depthINIData = NULL;

    delete (m_calData);
    m_calData = NULL;
    
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
CameraItof::processFrame(uint8_t * /*rawFrame*/, uint16_t * /*captureData*/,
                         uint8_t * /*head*/, const uint16_t /*embed_height*/,
                         const uint16_t /*embed_width*/,
                         aditof::FrameDetails & /*frameDetails*/) {
    // TO DO

    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::getCurrentModeInfo(ModeInfo::modeInfo &info) {
    using namespace aditof;
    Status status = Status::OK;
    uint8_t convertedMode;

    status = convertCameraMode(m_details.mode, &convertedMode);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return aditof::Status::GENERIC_ERROR;
    }

    ModeInfo *pModeInfo = ModeInfo::getInstance();
    if (pModeInfo && 0 <= convertedMode && convertedMode < pModeInfo->getNumModes()) {
        info = pModeInfo->getModeInfo(convertedMode);
        return Status::OK;
    } 
    return Status::GENERIC_ERROR;
}

aditof::Status CameraItof::cleanupTempFiles() {
     using namespace aditof;
    Status status = Status::OK;
    for (const std::string& filename : m_tempFiles) {
        if( std::remove( filename.c_str() ) != 0 ) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    m_tempFiles.clear();
    return status;
}

aditof::Status CameraItof::powerUp() {
    // TO DO
//defined in device_interface.h -> depth_sensor_interface.h
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::powerDown() {
    // TO DO
//defined in device_interface.h -> depth_sensor_interface.h
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::setCameraSyncMode(uint8_t mode, uint8_t level) {
//defined in device_interface.h -> depth_sensor_interface.h
    //return m_depthSensor->setCameraSyncMode(mode, level);
        return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::loadModuleData() {
  /*  using namespace aditof;
    Status status = Status::OK;

    cleanupTempFiles();
//EepromInterface -> StorageInterface
    std::shared_ptr<EepromInterface> eeprom = aditof::EepromFactory::getInstance().getDevice(m_eepromDeviceName);
    if (eeprom == nullptr) {
        LOG(ERROR) << "Undefined module memory device";
        return Status::GENERIC_ERROR;
    }

    std::string tempJsonFile;
    ModuleMemory flashLoader(m_depthSensor, eeprom);
    flashLoader.readModuleData(tempJsonFile, m_tempFiles);

    m_depthSensor->cameraReset();

    if (!tempJsonFile.empty()) {
        return initialize(tempJsonFile);
    } else {
        LOG(ERROR) << "Error loading module data";
        return Status::GENERIC_ERROR;
    }*/
        return aditof::Status::UNAVAILABLE;
}
