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
#include "aditof/frame.h"
#include "aditof/frame_operations.h"
#include "aditof_internal.h"

#include "cJSON.h"
#include "calibration_itof.h"
#include "crc.h"
#include "module_memory.h"
#include "tofi/floatTolin.h"
#include "tofi/tofi_config.h"
#include "utils.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <chrono>
#include <iostream>
#include <iterator>
#include <thread>
#include <vector>

CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &tSensors,
    const std::string &ubootVersion, const std::string &kernelVersion,
    const std::string &sdCardImageVersion)
    : m_depthSensor(depthSensor), m_devStarted(false),
      m_eepromInitialized(false), m_adsd3500Enabled(false),
      m_loadedConfigData(false), m_xyzEnabled(false), m_xyzSetViaControl(false),
      m_modechange_framedrop_count(0), m_tempFiles{}, m_cameraFps(0),
      m_fsyncMode(1), m_adsd3500ImagerType(0), m_modesVersion(0) {

    FloatToLinGenerateTable();

    m_details.mode = "sr-native";
    m_details.cameraId = "";
    m_details.uBootVersion = ubootVersion;
    m_details.kernelVersion = kernelVersion;
    m_details.sdCardImageVersion = sdCardImageVersion;

    // Define some of the controls of this camera
    m_controls.emplace("initialization_config", "");
    m_controls.emplace("syncMode", "0, 0");
    m_controls.emplace("saveModuleCCB", "");
    m_controls.emplace("saveModuleCFG", "");
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
    if (sensorName == "adsd3500") {
        m_adsd3500Enabled = true;
    } else if (sensorName == "offline") {
        m_isOffline = true;
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
        if (!m_adsd3500Enabled) {
            LOG(WARNING) << "Could not find " << m_eepromDeviceName
                         << " while looking for storage";
        }
        m_eeprom = NULL;
    } else {
        m_eeprom = *eeprom_iter;
    }

    // Additional controls
    if (m_adsd3500Enabled) {
        m_controls.emplace("updateAdsd3500Firmware", "");
        m_controls.emplace("imagerType", "1");
    }

    m_adsd3500_master = true;
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
    Status status = Status::OK;

    LOG(INFO) << "Initializing camera";

    // Setting up the UVC filters, samplegrabber interface, Video renderer and filters
    // Setting UVC mediaformat and Running the stream is done once mode is set
    if (!m_devStarted) {
        status = m_depthSensor->open();
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to open device";
            return status;
        }
        m_devStarted = true;
    }

    //get intrinsics for adsd3500 TO DO: check endianess of intrinsics
    if (m_adsd3500Enabled || m_isOffline) {

        // get imager type that is used toghether with ADSD3500
        std::string controlValue;
        status = m_depthSensor->getControl("imagerType", controlValue);
        if (status == Status::OK) {
            if (controlValue == "1" || controlValue == "2") {
                m_adsd3500ImagerType = std::stoi(controlValue);
            } else {
                LOG(ERROR) << "Unkown imager type: " << controlValue;
                return Status::UNAVAILABLE;
            }
        } else {
            LOG(ERROR) << "Failed to read the imager type";
            return Status::UNAVAILABLE;
        }

        // get CCB version (this tells whether we're using old or new modes)
        status = m_depthSensor->getControl("modeInfoVersion", controlValue);
        if (status == Status::OK) {
            if (controlValue == "0" || controlValue == "1" ||
                controlValue == "2") {
                m_modesVersion = std::stoi(controlValue);
            } else {
                LOG(ERROR) << "Unkown CCB version: " << controlValue;
            }
        } else {
            LOG(ERROR) << "Failed to read the CCB version";
        }

        // If depth sensor knows the modes version, use it, otherwise fallback to old workaround
        if (m_modesVersion != 0) {
            if (m_adsd3500ImagerType == 1) {
                if (m_modesVersion == 1) {
                    m_modesVersion = 0;
                } else if (m_modesVersion == 2) {
                    m_modesVersion = 2;
                }
            } else if (m_adsd3500ImagerType == 2) {
                if (m_modesVersion == 1) {
                    m_modesVersion = 0;

                } else if (m_modesVersion == 2) {
                    m_modesVersion = 2;
                }
            }
        } else { // The depth sensor doesn't know the modes version. Use the dealias info from NVM to figure it out
            if (m_adsd3500ImagerType == 1) { // Find for Crosby
                uint8_t tempDealiasParams[32] = {0};
                tempDealiasParams[0] = 1;

                TofiXYZDealiasData tempDealiasStruct;
                // We know by default getInstance points to new modes of ADSD3500
                uint16_t width = ModeInfo::getInstance()->getModeInfo(1).width;
                uint16_t height =
                    ModeInfo::getInstance()->getModeInfo(1).height;

                // We read dealias parameters to find out the width and height for mode 1
                status = m_depthSensor->adsd3500_read_payload_cmd(
                    0x02, tempDealiasParams, 32);
                if (status != Status::OK) {
                    LOG(ERROR)
                        << "Failed to read dealias parameters for adsd3500!";
                    return status;
                }

                memcpy(&tempDealiasStruct, tempDealiasParams,
                       sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));

                // Old modes had "lt_bin" as mode 1 with a resolution of width=320, height=288.
                if (tempDealiasStruct.n_rows != width &&
                    tempDealiasStruct.n_cols != height) {
                    m_modesVersion = 0;
                } else {
                    m_modesVersion = 2;
                }
            } else if (m_adsd3500ImagerType == 2) { // Find for Tembin
                int modeToTest =
                    0; // We are looking at width and height for mode 0
                uint8_t tempDealiasParams[32] = {0};
                tempDealiasParams[0] = modeToTest;

                TofiXYZDealiasData tempDealiasStruct;
                uint16_t width = 640;
                uint16_t height = 512;

                // We read dealias parameters to find out the width and height for mode 0
                status = m_depthSensor->adsd3500_read_payload_cmd(
                    0x02, tempDealiasParams, 32);
                if (status != Status::OK) {
                    LOG(ERROR)
                        << "Failed to read dealias parameters for adsd3500!";
                    return status;
                }

                memcpy(&tempDealiasStruct, tempDealiasParams,
                       sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));

                // If old modes, there won't be a width=512 and height=640 for mode 0. It would be only for mode 5 ('vga').
                if (tempDealiasStruct.n_rows == width &&
                    tempDealiasStruct.n_cols == height) {
                    m_modesVersion = 2;
                } else {
                    m_modesVersion = 0;
                }
            }
        }
        // Check weather new mixed modes are present, in case no, switch back to simple new modes
        if (m_modesVersion == 2) {
            int modeToTest = 5; // We are looking at width and height for mode 5
            uint8_t tempDealiasParams[32] = {0};
            tempDealiasParams[0] = modeToTest;

            TofiXYZDealiasData tempDealiasStruct;
            uint16_t width1 = 512;
            uint16_t height1 = 512;

            uint16_t width2 = 320;
            uint16_t height2 = 256;

            // We read dealias parameters to find out the width and height for mode 5
            status = m_depthSensor->adsd3500_read_payload_cmd(
                0x02, tempDealiasParams, 32);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read dealias parameters for adsd3500!";
                return status;
            }

            memcpy(&tempDealiasStruct, tempDealiasParams,
                   sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));

            // If mixed modes don't have accurate dimensions, switch back to simple new modes table
            if ((tempDealiasStruct.n_rows == width1 &&
                 tempDealiasStruct.n_cols == height1) ||
                (tempDealiasStruct.n_rows == width2 &&
                 tempDealiasStruct.n_cols == height2)) {
                m_modesVersion = 3;
            }
        }

        ModeInfo::getInstance()->setImagerTypeAndModeVersion(
            m_adsd3500ImagerType, m_modesVersion);
        setControl("imagerType", std::to_string(m_adsd3500ImagerType));
        status = m_depthSensor->setControl("modeInfoVersion",
                                           std::to_string(m_modesVersion));
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set mode versioning on target";
            return status;
        }

        m_depthSensor->getAvailableFrameTypes(m_availableSensorFrameTypes);

        for (auto availableFrameTypes : m_availableSensorFrameTypes) {
            uint8_t intrinsics[56] = {0};
            uint8_t dealiasParams[32] = {0};
            TofiXYZDealiasData dealiasStruct;
            //the first element of readback_data for adsd3500_read_payload is used for the custom command
            //it will be overwritten by the returned data
            uint8_t mode = ModeInfo::getInstance()
                               ->getModeInfo(availableFrameTypes.type)
                               .mode;

            intrinsics[0] = mode;
            dealiasParams[0] = mode;
            //hardcoded function values to return intrinsics
            status =
                m_depthSensor->adsd3500_read_payload_cmd(0x01, intrinsics, 56);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read intrinsics for adsd3500!";
                return status;
            }

            //hardcoded function values to return dealias parameters
            status = m_depthSensor->adsd3500_read_payload_cmd(
                0x02, dealiasParams, 32);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read dealias parameters for adsd3500!";
                return status;
            }

            memcpy(&dealiasStruct, dealiasParams,
                   sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));
            memcpy(&dealiasStruct.camera_intrinsics, intrinsics,
                   sizeof(CameraIntrinsics));

            m_xyz_dealias_data[mode] = dealiasStruct;
            memcpy(&m_details.intrinsics, &dealiasStruct.camera_intrinsics,
                   sizeof(CameraIntrinsics));
        }

        std::string fwVersion;
        std::string fwHash;
        status = adsd3500GetFirmwareVersion(fwVersion, fwHash);

        if (status == Status::OK) {
            LOG(INFO) << "Current adsd3500 firmware version is: "
                      << m_adsd3500FwGitHash.first;
            LOG(INFO) << "Current adsd3500 firmware git hash is: "
                      << m_adsd3500FwGitHash.second;
        } else
            return status;
    }

    if (m_eeprom) {
        void *handle;
        status = m_depthSensor->getHandle(&handle);
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

    //Populate the data from the json file provided
    status = parseJsonFileContent();
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to parse Json file!";
        return status;
    }

    status = loadModuleData();
    if (status != Status::OK) {
        LOG(INFO) << "No CCB/CFG data found in camera module,";
        LOG(INFO) << "Loading calibration(ccb) and configuration(cfg) data "
                     "from JSON config file...";
    } else {
        //CCB and CFG files will be taken from module memory if
        //they are not passed in the initialization_config json file
        status = parseJsonFileContent();
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to parse Json file!";
            return status;
        }
    }

    aditof::Status configStatus = loadConfigData();
    if (configStatus == aditof::Status::OK) {
        m_loadedConfigData = true;
    } else {
        LOG(ERROR) << "loadConfigData failed";
        return Status::GENERIC_ERROR;
    }

    ////check first mode to set ModeInfo table version for non adsd3500
    if (m_loadedConfigData && !m_adsd3500Enabled && !m_isOffline) {
        TofiXYZDealiasData tempDealiasStruct[11];
        uint16_t width = ModeInfo::getInstance()->getModeInfo(1).width;
        uint16_t height = ModeInfo::getInstance()->getModeInfo(1).height;

        uint32_t err =
            GetXYZ_DealiasData((ConfigFileData *)&m_calData, tempDealiasStruct);
        if (err != ADI_TOFI_SUCCESS) {
            LOG(ERROR) << "Failed to get dealias data from ccb!";
            return Status::GENERIC_ERROR;
        }

        if (tempDealiasStruct[1].n_rows != width &&
            tempDealiasStruct[1].n_cols != height) {
            ModeInfo::getInstance()->setImagerTypeAndModeVersion(1, 0);
            status = m_depthSensor->setControl("modeInfoVersion", "0");
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set target mode info for adsd3100!";
                return status;
            }
        } else {
            ModeInfo::getInstance()->setImagerTypeAndModeVersion(1, 1);
            status = m_depthSensor->setControl("modeInfoVersion", "1");
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set target mode info for adsd3100!";
                return status;
            }
        }

        m_depthSensor->getAvailableFrameTypes(m_availableSensorFrameTypes);
    }

    if (m_adsd3500Enabled) {
        status = adsd3500SetToggleMode(m_fsyncMode);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set fsync mode";
            return status;
        }
    }

    LOG(INFO) << "Camera initialized";

    return Status::OK;
}

aditof::Status CameraItof::start() {
    using namespace aditof;
    Status status = Status::OK;

    if (m_adsd3500Enabled || m_isOffline) {
        m_CameraProgrammed = true;
        status = m_depthSensor->start();
        if (Status::OK != status) {
            LOG(ERROR) << "Error starting adsd3500.";
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
        LOG(INFO) << "USEQ running. Skip CFG & CCB "
                     "programming step\n";
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
    aditof::Status status;

    status = m_depthSensor->stop();
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to stop camera!";
    }

    return status;
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

aditof::Status CameraItof::setFrameType(const std::string &frameType) {
    using namespace aditof;
    Status status = Status::OK;

    auto frameTypeIt = std::find_if(
        m_availableSensorFrameTypes.begin(), m_availableSensorFrameTypes.end(),
        [&frameType](const DepthSensorFrameType &d) {
            return (d.type == frameType);
        });

    if (frameTypeIt == m_availableSensorFrameTypes.end()) {
        LOG(WARNING) << "Frame type: " << frameType
                     << " not supported by camera";
        return Status::INVALID_ARGUMENT;
    }

    if (m_ini_depth_map.size() > 1) {
        m_ini_depth = m_ini_depth_map[frameType];
    }

    if (m_details.connection == ConnectionType::USB) {
        status = m_depthSensor->adsd3500_reset();
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to reset the camera!";
            return status;
        }
    }

    if (frameType == "pcm-native") {
        m_pcmFrame = true;
    } else {
        m_pcmFrame = false;
    }

    getKeyValuePairsFromIni(m_ini_depth, m_iniKeyValPairs);
    setAdsd3500WithIniParams(m_iniKeyValPairs);
    configureSensorFrameType();
    setMode(frameType);
    LOG(INFO) << "Using ini file: " << m_ini_depth;

    status = m_depthSensor->setFrameType(*frameTypeIt);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set frame type";
        return status;
    }

    // Store the frame details in camera details
    m_details.frameType.type = (*frameTypeIt).type;
    // TO DO: m_details.frameType.cameraMode =
    m_details.frameType.width =
        ModeInfo::getInstance()->getModeInfo(frameType).width;
    m_details.frameType.height =
        ModeInfo::getInstance()->getModeInfo(frameType).height;
    if (!m_adsd3500Enabled) {
        m_details.frameType.totalCaptures =
            ModeInfo::getInstance()->getModeInfo(frameType).subframes;
    } else {
        m_details.frameType.totalCaptures = 1;
    }
    m_details.frameType.dataDetails.clear();
    for (const auto &item : (*frameTypeIt).content) {
        if (item.type == "xyz" && !m_xyzEnabled) {
            continue;
        }

        FrameDataDetails fDataDetails;
        fDataDetails.type = item.type;
        fDataDetails.width = item.width;
        fDataDetails.height = item.height;
        fDataDetails.subelementSize = sizeof(uint16_t);
        fDataDetails.subelementsPerElement = 1;

        if (item.type == "xyz") {
            fDataDetails.subelementsPerElement = 3;
        }

        m_details.frameType.dataDetails.emplace_back(fDataDetails);
    }

    if (m_controls["enableDepthCompute"] == "on" && !m_pcmFrame &&
        ((m_details.frameType.totalCaptures > 1) || m_adsd3500Enabled ||
         m_isOffline)) {
        status = initComputeLibrary();
        if (Status::OK != status) {
            LOG(ERROR) << "Initializing compute libraries failed.";
            return Status::GENERIC_ERROR;
        }
    } else {
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

aditof::Status CameraItof::getFrameTypeNameFromId(int id,
                                                  std::string &name) const {
    using namespace aditof;
    Status status = Status::OK;

    ModeInfo::modeInfo info = ModeInfo::getInstance()->getModeInfo(id);
    if (!info.mode_name.empty()) {
        name = info.mode_name;
    } else {
        status = Status::INVALID_ARGUMENT;
    }

    return status;
}

aditof::Status setAttributesByMode(aditof::Frame &frame,
                                   const ModeInfo::modeInfo &modeInfo) {
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

    if (frame == nullptr) {
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

    if (!m_adsd3500Enabled) {
        frame->getAttribute("total_captures", totalCapturesStr);
        totalCaptures = std::atoi(totalCapturesStr.c_str());
    } else {
        totalCaptures = 1;
    }

    uint16_t *frameDataLocation = nullptr;
    if ((m_details.frameType.type == "pcm-native")) {
        frame->getData("ir", &frameDataLocation);
    } else if (m_details.frameType.type == "") {
        LOG(ERROR) << "Frame type not found!";
        return Status::INVALID_ARGUMENT;
    } else {
        frame->getData("raw", &frameDataLocation);
    }
    if (!frameDataLocation) {
        LOG(WARNING) << "getframe failed to allocated valid frame";
        return status;
    }

    status = m_depthSensor->getFrame(frameDataLocation);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from device";
    }

    if (!m_adsd3500Enabled && !m_isOffline &&
        (m_details.frameType.type != "pcm-native")) {
        for (unsigned int i = 0;
             i < (m_details.frameType.height * m_details.frameType.width *
                  totalCaptures);
             ++i) {
            frameDataLocation[i] = frameDataLocation[i] >> 4;
            frameDataLocation[i] =
                Convert11bitFloat2LinearVal(frameDataLocation[i]);
        }
    }

    if ((m_controls["enableDepthCompute"] == "on") && !m_pcmFrame &&
        ((totalCaptures > 1) || m_adsd3500Enabled || m_isOffline)) {

        if (NULL == m_tofi_compute_context) {
            LOG(ERROR) << "Depth compute libray not initialized";
            return Status::GENERIC_ERROR;
        }
        uint16_t *tempDepthFrame = m_tofi_compute_context->p_depth_frame;
        uint16_t *tempAbFrame = m_tofi_compute_context->p_ab_frame;
        uint16_t *tempXyzFrame =
            (uint16_t *)m_tofi_compute_context->p_xyz_frame;
        // uint16_t *tempConfFrame =
        //     (uint16_t *)m_tofi_compute_context->p_conf_frame; // TO DO: Uncomment this and figure out why depth compute is crashing

        frame->getData("depth", &m_tofi_compute_context->p_depth_frame);
        frame->getData("ir", &m_tofi_compute_context->p_ab_frame);

        // uint16_t *confFrame;
        // frame->getData("conf", &confFrame);
        // m_tofi_compute_context->p_conf_frame = (float *)confFrame; // TO DO: Uncomment this and figure out why depth compute is crashing

        if (m_xyzEnabled) {
            uint16_t *xyzFrame;
            frame->getData("xyz", &xyzFrame);
            m_tofi_compute_context->p_xyz_frame = (int16_t *)xyzFrame;
        }

        uint32_t ret =
            TofiCompute(frameDataLocation, m_tofi_compute_context, NULL);

        if (ret != ADI_TOFI_SUCCESS) {
            LOG(ERROR) << "TofiCompute failed";
            return Status::GENERIC_ERROR;
        }

        m_tofi_compute_context->p_depth_frame = tempDepthFrame;
        m_tofi_compute_context->p_ab_frame = tempAbFrame;
        m_tofi_compute_context->p_xyz_frame = (int16_t *)tempXyzFrame;
        // m_tofi_compute_context->p_conf_frame = (float *)tempConfFrame;

        if (m_adsd3500Enabled && m_abEnabled && (m_adsd3500ImagerType == 1) &&
                (m_abBitsPerPixel < 16) &&
                (m_details.frameType.type == "lr-native" ||
                 m_details.frameType.type == "sr-native") ||
            m_isOffline) {
            uint16_t *mpAbFrame;
            frame->getData("ir", &mpAbFrame);

            //TO DO: shift with 4 because we use only 12 bits
            uint8_t bitsToShift = 16 - m_abBitsPerPixel;
            for (unsigned int i = 0;
                 i < (m_details.frameType.height * m_details.frameType.width);
                 ++i) {
                mpAbFrame[i] = mpAbFrame[i] >> bitsToShift;
            }
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
            if (m_adsd3500Enabled) {
                status = readAdsd3500CCB();
                if (status != Status::OK) {
                    LOG(ERROR) << "Failed to load ccb from adsd3500 module!";
                    return Status::GENERIC_ERROR;
                }
            }
            return saveCCBToFile(value);
        } else if (control == "saveModuleCFG") {
            return saveCFGToFile(value);
        } else if (control == "enableXYZframe") {
            if (value == "on")
                return enableXYZframe(true);
            else if (value == "off")
                return enableXYZframe(false);
            else
                return Status::INVALID_ARGUMENT;
        } else if (control == "updateAdsd3500Firmware") {
            return updateAdsd3500Firmware(value);
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

    status = ModeInfo::getInstance()->convertCameraMode(m_details.mode,
                                                        convertedMode);

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Invalid mode!";
        return aditof::Status::GENERIC_ERROR;
    }

    if (m_loadedConfigData || m_adsd3500Enabled || m_isOffline) {
        ConfigFileData calData = {m_calData.p_data, m_calData.size};
        uint32_t status = ADI_TOFI_SUCCESS;

        if (!m_ini_depth.empty()) {
            size_t dataSize = m_depthINIData.size;
            unsigned char *pData = m_depthINIData.p_data;

            if (m_depthINIDataMap.size() > 1) {
                dataSize = m_depthINIDataMap[m_ini_depth].size;
                pData = m_depthINIDataMap[m_ini_depth].p_data;
            }

            uint8_t *tempDataParser = new uint8_t[dataSize];
            memcpy(tempDataParser, pData, dataSize);
            ConfigFileData depth_ini = {tempDataParser, dataSize};

            if (m_adsd3500Enabled || m_isOffline) {
                m_tofi_config = InitTofiConfig_isp((ConfigFileData *)&depth_ini,
                                                   convertedMode, &status,
                                                   m_xyz_dealias_data);
            } else {
                if (calData.p_data != NULL) {
                    m_tofi_config = InitTofiConfig(&calData, NULL, &depth_ini,
                                                   convertedMode, &status);
                } else {
                    LOG(ERROR) << "Failed to get calibration data";
                }
            }

            delete[] tempDataParser;

        } else {
            m_tofi_config =
                InitTofiConfig(&calData, NULL, NULL, convertedMode, &status);
        }

        if ((m_tofi_config == NULL) ||
            (m_tofi_config->p_tofi_cal_config == NULL) ||
            (status != ADI_TOFI_SUCCESS)) {
            LOG(ERROR) << "InitTofiConfig failed";
            return aditof::Status::GENERIC_ERROR;

        } else {
            m_tofi_compute_context =
                InitTofiCompute(m_tofi_config->p_tofi_cal_config, &status);
            if (m_tofi_compute_context == NULL || status != ADI_TOFI_SUCCESS) {
                LOG(ERROR) << "InitTofiCompute failed";
                return aditof::Status::GENERIC_ERROR;
            }
        }
    } else {
        LOG(ERROR) << "Could not initialize compute library because config "
                      "data hasn't been loaded";
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
    freeConfigData();

    if (m_ini_depth_map.size() > 0) {
        for (auto it = m_ini_depth_map.begin(); it != m_ini_depth_map.end();
             ++it) {
            m_depthINIDataMap.emplace(
                it->second,
                LoadFileContents(const_cast<char *>(it->second.c_str())));
        }
    } else {
        if (!m_ini_depth.empty()) {
            m_depthINIData =
                LoadFileContents(const_cast<char *>(m_ini_depth.c_str()));
        }
    }

    if (!m_ccb_calibrationFile.empty()) {
        m_calData =
            LoadFileContents(const_cast<char *>(m_ccb_calibrationFile.c_str()));
    } else {
        m_calData.p_data = NULL;
        m_calData.size = 0;
    }

    return aditof::Status::OK;
}

void CameraItof::freeConfigData(void) {
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

aditof::Status CameraItof::processFrame(uint8_t *rawFrame,
                                        uint16_t *captureData, uint8_t *head,
                                        const uint16_t embed_height,
                                        const uint16_t embed_width,
                                        aditof::Frame *frame) {
    using namespace aditof;
    Status status = Status::OK;

    // Read header data and process image
    // uint16_t REG_CAPTURE_ID = 0;
    uint16_t FrameWidth = 0;
    uint16_t FrameHeight = 0;
    uint16_t FrameNum = 0;
    // uint8_t totalCaptures = 0;
    uint8_t captureID = 0;
    // uint16_t chipID = 0;
    // uint16_t REG_MODE_ID_CURR = 0;
    // uint8_t Mode = 0;

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
    // uint64_t rawSubFrameSize =
    //     ((embed_height * embed_width * 2) / (totalCaptures + 1)) - 128;
    // uint64_t subFrameSize =
    //     FrameWidth * FrameHeight; // capture size without header
    uint16_t *p = (uint16_t *)rawFrame;

    for (int i = 0; i < FrameNum * FrameWidth * FrameHeight; ++i) {
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

    ModeInfo *pModeInfo = ModeInfo::getInstance();
    if (pModeInfo) {
        uint8_t convertedMode;
        status = pModeInfo->convertCameraMode(m_details.mode, convertedMode);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Invalid mode!";
            return aditof::Status::GENERIC_ERROR;
        }
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
        if (std::remove(filename.c_str()) != 0) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    filename = m_tempFiles.ccbFile;
    if (!filename.empty()) {
        if (std::remove(filename.c_str()) != 0) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    filename = m_tempFiles.cfgFile;
    if (!filename.empty()) {
        if (std::remove(filename.c_str()) != 0) {
            LOG(WARNING) << "Failed temp file delete: " << filename;
            status = Status::GENERIC_ERROR;
        }
    }

    m_tempFiles = {};

    return status;
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

    if (m_adsd3500Enabled) {
        return status;
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
    int prefixCount =
        2; // I'm asuming the first 2 characters are "D:" which we don't need
    std::string shortName = "";
    if (serialNumber != "")
        shortName = serialNumber.substr(prefixCount,
                                        serialNumber.find(" ") - prefixCount);
    else
        shortName =
            "default"; //This name is given when there has been an error on CRC
    m_details.cameraId = shortName;

    // m_depthSensor->cameraReset(); TO DO: figure out if this is required or how to do the reset since there is currenlty no cameraReset() in DepthSensorInterface

    if (!tempJsonFile.empty()) {
        setControl("initialization_config", tempJsonFile);
        return status;
    } else {
        LOG(ERROR) << "Error loading module data";
        return Status::GENERIC_ERROR;
    }
}

aditof::Status CameraItof::applyCalibrationToFrame(uint16_t *frame,
                                                   const unsigned int mode) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::saveCCBToFile(const std::string &filePath) const {
    if (filePath.empty()) {
        LOG(ERROR) << "File path where CCB should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    if (m_tempFiles.ccbFile.empty()) {
        LOG(ERROR) << "CCB files is unavailable. Perhaps CCB content was not "
                      "read from module.";
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

#pragma pack(push, 1)
typedef union {
    uint8_t cmd_header_byte[16];
    struct {
        uint8_t id8;                // 0xAD
        uint16_t chunk_size16;      // 256 is flash page size
        uint8_t cmd8;               // 0x04 is the CMD for fw upgrade
        uint32_t total_size_fw32;   // 4 bytes (total size of firmware)
        uint32_t header_checksum32; // 4 bytes header checksum
        uint32_t crc_of_fw32;       // 4 bytes CRC of the Firmware Binary
    };
} cmd_header_t;
#pragma pack(pop)

aditof::Status CameraItof::updateAdsd3500Firmware(const std::string &filePath) {
    using namespace aditof;
    Status status = Status::OK;

    m_fwUpdated = false;
    m_adsd3500Status = Adsd3500Status::OK;
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_fwUpdated = true;
    };
    status = m_depthSensor->adsd3500_register_interrupt_callback(cb);
    bool interruptsAvailable = (status == Status::OK);

    // Read Chip ID in STANDARD mode
    uint16_t chip_id;
    status = m_depthSensor->adsd3500_read_cmd(0x0112, &chip_id);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read adsd3500 chip id!";
        return status;
    }

    LOG(INFO) << "The readback chip ID is: " << chip_id;

    // Switch to BURST mode.
    status = m_depthSensor->adsd3500_write_cmd(0x0019, 0x0000);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch to burst mode!";
        return status;
    }

    // Send FW content, each chunk is 256 bytes
    const int flashPageSize = 256;

    // Read the firmware binary file
    std::ifstream fw_file(filePath, std::ios::binary);
    // copy all data into buffer
    std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(fw_file), {});

    uint32_t fw_len = buffer.size();
    uint8_t *fw_content = buffer.data();
    cmd_header_t fw_upgrade_header;
    fw_upgrade_header.id8 = 0xAD;
    fw_upgrade_header.chunk_size16 = 0x0100; // 256=0x100
    fw_upgrade_header.cmd8 = 0x04;           // FW Upgrade CMD = 0x04
    fw_upgrade_header.total_size_fw32 = fw_len;
    fw_upgrade_header.header_checksum32 = 0;

    for (int i = 1; i < 8; i++) {
        fw_upgrade_header.header_checksum32 +=
            fw_upgrade_header.cmd_header_byte[i];
    }

    uint32_t res = crcFast(fw_content, fw_len, true) ^ 0xFFFFFFFF;
    fw_upgrade_header.crc_of_fw32 = ~res;

    status = m_depthSensor->adsd3500_write_payload(
        fw_upgrade_header.cmd_header_byte, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to send fw upgrade header";
        return status;
    }

    int packetsToSend;
    if ((fw_len % flashPageSize) != 0) {
        packetsToSend = (fw_len / flashPageSize + 1);
    } else {
        packetsToSend = (fw_len / flashPageSize);
    }

    uint8_t data_out[flashPageSize];

    for (int i = 0; i < packetsToSend; i++) {
        int start = flashPageSize * i;
        int end = flashPageSize * (i + 1);

        for (int j = start; j < end; j++) {
            if (j < fw_len) {
                data_out[j - start] = fw_content[j];
            } else {
                // padding with 0x00
                data_out[j - start] = 0x00;
            }
        }
        status = m_depthSensor->adsd3500_write_payload(data_out, flashPageSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to send packet number " << i << " out of "
                       << packetsToSend << " packets!";
            return status;
        }

        if (i % 25 == 0) {
            LOG(INFO) << "Succesfully sent " << i << " out of " << packetsToSend
                      << " packets";
        }
    }

    //Commands to switch back to standard mode
    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = m_depthSensor->adsd3500_write_payload(
        switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch adsd3500 to standard mode!";
        return status;
    }

    if (interruptsAvailable) {
        LOG(INFO) << "Waiting for ADSD3500 to update itself";
        int secondsTimeout = 60;
        int secondsWaited = 0;
        int secondsWaitingStep = 1;
        while (!m_fwUpdated && secondsWaited < secondsTimeout) {
            LOG(INFO) << ".";
            std::this_thread::sleep_for(
                std::chrono::seconds(secondsWaitingStep));
            secondsWaited += secondsWaitingStep;
        }
        LOG(INFO) << "Waited: " << secondsWaited << " seconds";
        m_depthSensor->adsd3500_register_interrupt_callback(nullptr);
        if (!m_fwUpdated && secondsWaited >= secondsTimeout) {
            LOG(WARNING) << "Adsd3500 firmware updated has timeout after: "
                         << secondsWaited << "seconds";
            return aditof::Status::GENERIC_ERROR;
        }

        if (m_adsd3500Status == Adsd3500Status::OK ||
            m_adsd3500Status == Adsd3500Status::FIRMWARE_UPDATE_COMPLETE) {
            LOG(INFO) << "Adsd3500 firmware updated succesfully!";
        } else {
            LOG(ERROR) << "Adsd3500 firmware updated but with error: "
                       << (int)m_adsd3500Status;
        }
    } else {
        LOG(INFO) << "Adsd3500 firmware updated succesfully! Waiting 60 "
                     "seconds since interrupts support was not detected.";
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }

    return aditof::Status::OK;
}

aditof::Status CameraItof::readAdsd3500CCB() {
    using namespace aditof;
    Status status = Status::OK;

    uint8_t ccbHeader[16] = {0};
    ccbHeader[0] = 1;

    //For this case adsd3500 will remain in burst mode
    //A manuall switch to standard mode will be required at the end of the function
    status = m_depthSensor->adsd3500_read_payload_cmd(0x13, ccbHeader, 16);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get ccb command header";
        return status;
    }

    uint16_t chunkSize;
    uint32_t ccbFileSize;
    uint32_t crcOfCCB;
    uint16_t numOfChunks;

    memcpy(&chunkSize, ccbHeader + 1, 2);
    memcpy(&ccbFileSize, ccbHeader + 4, 4);
    memcpy(&crcOfCCB, ccbHeader + 12, 4);

    numOfChunks = ccbFileSize / chunkSize;
    uint8_t *ccbContent = new uint8_t[ccbFileSize];

    for (int i = 0; i < numOfChunks; i++) {
        status = m_depthSensor->adsd3500_read_payload(
            ccbContent + i * chunkSize, chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << i << " out of "
                       << numOfChunks + 1 << " chunks for adsd3500!";
            return status;
        }

        if (i % 20 == 0) {
            LOG(INFO) << "Succesfully read chunk number " << i << " out of "
                      << numOfChunks + 1 << " chunks for adsd3500!";
        }
    }

    //read last chunk. smaller size than the rest
    if (ccbFileSize % chunkSize != 0) {
        status = m_depthSensor->adsd3500_read_payload(
            ccbContent + numOfChunks * chunkSize, ccbFileSize % chunkSize);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to read chunk number " << numOfChunks + 1
                       << " out of " << numOfChunks + 1
                       << " chunks for adsd3500!";
            return status;
        }
    }

    //Commands to switch back to standard mode
    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = m_depthSensor->adsd3500_write_payload(
        switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to switch adsd3500 to standard mode!";
        return status;
    }

    LOG(INFO) << "Succesfully read ccb from adsd3500. Checking crc...";

    uint32_t computedCrc =
        crcFast(ccbContent, ccbFileSize - 4, true) ^ 0xFFFFFFFF;

    if (crcOfCCB != ~computedCrc) {
        LOG(ERROR) << "Invalid crc for ccb read from memory!";
        return Status::GENERIC_ERROR;
    } else {
        LOG(INFO) << "Crc of ccb is valid.";
    }

    std::ofstream tempFile;
    std::string fileName = "temp_ccb.ccb";

    //remove the trailling 4 bytes containing the crc
    std::string fileContent = std::string((char *)ccbContent, ccbFileSize - 4);
    tempFile.open(fileName, std::ios::binary);

    tempFile << fileContent;

    m_tempFiles.ccbFile = fileName;
    delete[] ccbContent;
    tempFile.close();

    return status;
}

void CameraItof::configureSensorFrameType() {
    std::string value;

    auto it = m_iniKeyValPairs.find("bitsInPhaseOrDepth");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        if (value == "16")
            value = "6";
        else if (value == "14")
            value = "5";
        else if (value == "12")
            value = "4";
        else if (value == "10")
            value = "3";
        else
            value = "2";
        m_depthSensor->setControl("phaseDepthBits", value);
    } else {
        LOG(WARNING) << "bitsInPhaseOrDepth was not found in .ini file";
    }

    it = m_iniKeyValPairs.find("bitsInConf");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        if (value == "8")
            value = "2";
        else if (value == "4")
            value = "1";
        else
            value = "0";
        m_depthSensor->setControl("confidenceBits", value);
    } else {
        LOG(WARNING) << "bitsInConf was not found in .ini file";
    }

    it = m_iniKeyValPairs.find("bitsInAB");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        m_abEnabled = 1;
        m_abBitsPerPixel = std::stoi(value);
        if (value == "16")
            value = "6";
        else if (value == "14")
            value = "5";
        else if (value == "12")
            value = "4";
        else if (value == "10")
            value = "3";
        else if (value == "8")
            value = "2";
        else {
            value = "0";
            m_abEnabled = 0;
        }
        m_depthSensor->setControl("abBits", value);
    } else {
        LOG(WARNING) << "bitsInAB was not found in .ini file";
    }

    it = m_iniKeyValPairs.find("partialDepthEnable");
    if (it != m_iniKeyValPairs.end()) {
        std::string en = (it->second == "0") ? "1" : "0";
        m_depthSensor->setControl("depthEnable", en);
        m_depthSensor->setControl("abAveraging", en);
    } else {
        LOG(WARNING) << "partialDepthEnable was not found in .ini file";
    }

    // XYZ set through camera control takes precedence over the setting from .ini file
    if (!m_xyzSetViaControl) {
        it = m_iniKeyValPairs.find("xyzEnable");
        if (it != m_iniKeyValPairs.end()) {
            m_xyzEnabled = !(it->second == "0");
        } else {
            LOG(WARNING) << "xyzEnable was not found in .ini file";
        }
    }
}

aditof::Status CameraItof::parseJsonFileContent() {
    using namespace aditof;
    Status status = Status::OK;

    // Parse config.json
    std::string config = m_controls["initialization_config"];
    std::ifstream ifs(config.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    cJSON *config_json = cJSON_Parse(content.c_str());
    if (config_json != NULL) {
        // Get sensorfirmware file location
        const cJSON *json_sensorFirmware_file =
            cJSON_GetObjectItemCaseSensitive(config_json, "sensorFirmware");
        if (cJSON_IsString(json_sensorFirmware_file) &&
            (json_sensorFirmware_file->valuestring != NULL)) {
            if (m_sensorFirmwareFile.empty()) {
                // save firmware file location
                m_sensorFirmwareFile =
                    std::string(json_sensorFirmware_file->valuestring);
                LOG(INFO) << "Current sensor firmware is: "
                          << m_sensorFirmwareFile;
            } else {
                LOG(WARNING) << "Duplicate firmware file ignored: "
                             << json_sensorFirmware_file->valuestring;
            }
        }

        // Get calibration file location
        const cJSON *json_ccb_calibration_file =
            cJSON_GetObjectItemCaseSensitive(config_json, "CCB_Calibration");
        if (cJSON_IsString(json_ccb_calibration_file) &&
            (json_ccb_calibration_file->valuestring != NULL)) {
            if (m_ccb_calibrationFile.empty()) {
                // save calibration file location
                m_ccb_calibrationFile =
                    std::string(json_ccb_calibration_file->valuestring);
                LOG(INFO) << "Current calibration file is: "
                          << m_ccb_calibrationFile;
            } else {
                LOG(WARNING) << "Duplicate calibration file ignored: "
                             << json_ccb_calibration_file->valuestring;
            }
        }

        // Get optional eeprom type name
        const cJSON *eeprom_type_name =
            cJSON_GetObjectItemCaseSensitive(config_json, "MODULE_EEPROM_TYPE");
        if (cJSON_IsString(eeprom_type_name) &&
            (eeprom_type_name->valuestring != NULL)) {
            m_eepromDeviceName = eeprom_type_name->valuestring;
        }

        // Get depth ini file location
        const cJSON *json_depth_ini_file =
            cJSON_GetObjectItemCaseSensitive(config_json, "DEPTH_INI");
        if (cJSON_IsString(json_depth_ini_file) &&
            (json_depth_ini_file->valuestring != NULL)) {
            // store depth ini file location
            std::string mode;
            std::vector<std::string> iniFiles;

            if (m_ini_depth.empty()) {
                Utils::splitIntoTokens(
                    std::string(json_depth_ini_file->valuestring), ';',
                    iniFiles);
                if (iniFiles.size() > 1) {
                    for (const std::string &file : iniFiles) {
                        //extract last string that is after last underscore (e.g. 'mp' will be extracted from ini_file_mp)
                        size_t lastUnderscorePos = file.find_last_of("_");
                        if (lastUnderscorePos == std::string::npos) {
                            LOG(WARNING) << "File: " << file
                                         << " has no suffix that can be used "
                                            "to identify the mode";
                            continue;
                        }

                        size_t dotPos = file.find_last_of(".");
                        mode = file.substr(lastUnderscorePos + 1,
                                           dotPos - lastUnderscorePos - 1);
                        // TO DO: check is mode is supported by the camera

                        LOG(INFO) << "Found Depth ini file: " << file;
                        // Create map with mode name as key and path as value
                        m_ini_depth_map.emplace(mode, file);
                    }
                    // Set m_ini_depth to first map element
                    auto it = m_ini_depth_map.begin();
                    m_ini_depth = it->second;
                } else {
                    m_ini_depth = std::string(json_depth_ini_file->valuestring);
                }

                LOG(INFO) << "Current Depth ini file is: " << m_ini_depth;
            }
        }

        // Get fsync mode from config
        const cJSON *json_fsync_mode =
            cJSON_GetObjectItemCaseSensitive(config_json, "FSYNC_MODE");
        if (cJSON_IsString(json_fsync_mode) &&
            (json_fsync_mode->valuestring != NULL)) {
            m_fsyncMode = atoi(json_fsync_mode->valuestring);
        }

    } else if (!config.empty()) {
        LOG(ERROR) << "Couldn't parse config file: " << config.c_str();
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status CameraItof::adsd3500SetToggleMode(int mode) {
    /*mode = 2, adsd3500 fsync does not automatically toggle - Pin set as input (Slave)*/
    /*mode = 1, adsd3500 fsync automatically toggles at user specified framerate*/
    /*mode = 0, adsd3500 fsync does not automatically toggle*/
    using namespace aditof;
    Status status = Status::OK;

    status = m_depthSensor->adsd3500_write_cmd(0x0025, mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Unable to set FSYNC Toggle mode!";
        return status;
    }

    if (mode == 2) {
        m_adsd3500_master = false;
    }

    return status;
}

aditof::Status CameraItof::adsd3500ToggleFsync() {
    using namespace aditof;
    Status status = Status::OK;

    if (!m_adsd3500_master) {
        LOG(ERROR) << "ADSD3500 not set as master - cannot toggle FSYNC";
    } else {
        // Toggle Fsync
        status = m_depthSensor->adsd3500_write_cmd(0x0026, 0x0000);
        if (status != Status::OK) {
            LOG(ERROR) << "Unable to Toggle FSYNC!";
            return status;
        }
    }

    return status;
}

aditof::Status CameraItof::adsd3500GetFirmwareVersion(std::string &fwVersion,
                                                      std::string &fwHash) {
    using namespace aditof;
    Status status = Status::OK;
    uint8_t fwData[44] = {0};
    fwData[0] = uint8_t(1);

    status = m_depthSensor->adsd3500_read_payload_cmd(0x05, fwData, 44);
    if (status != Status::OK) {
        LOG(INFO) << "Failed to retrieve fw version and git hash for "
                     "adsd3500!";
        return status;
    }

    std::string fwv;

    fwv = std::to_string(fwData[0]) + '.' + std::to_string(fwData[1]) + '.' +
          std::to_string(fwData[2]) + '.' + std::to_string(fwData[3]);

    m_adsd3500FwGitHash =
        std::make_pair(fwv, std::string((char *)(fwData + 4), 40));

    fwVersion = m_adsd3500FwGitHash.first;
    fwHash = m_adsd3500FwGitHash.second;

    return status;
}

aditof::Status CameraItof::adsd3500SetABinvalidationThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0010, threshold);
}

aditof::Status CameraItof::adsd3500GetABinvalidationThreshold(int &threshold) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0015, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetConfidenceThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0011, threshold);
}
aditof::Status CameraItof::adsd3500GetConfidenceThreshold(int &threshold) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0016, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetJBLFfilterEnableState(bool enable) {
    return m_depthSensor->adsd3500_write_cmd(0x0013, enable ? 1 : 0);
}
aditof::Status CameraItof::adsd3500GetJBLFfilterEnableState(bool &enabled) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0017, reinterpret_cast<uint16_t *>(&enabled));
}

aditof::Status CameraItof::adsd3500SetJBLFfilterSize(int size) {
    return m_depthSensor->adsd3500_write_cmd(0x0014, size);
}
aditof::Status CameraItof::adsd3500GetJBLFfilterSize(int &size) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0018, reinterpret_cast<uint16_t *>(&size));
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMin(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0027, threshold);
}
aditof::Status CameraItof::adsd3500GetRadialThresholdMin(int &threshold) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0028, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMax(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0029, threshold);
}

aditof::Status CameraItof::adsd3500GetRadialThresholdMax(int &threshold) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0030, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetMIPIOutputSpeed(uint16_t speed) {
    return m_depthSensor->adsd3500_write_cmd(0x0031, speed);
}

aditof::Status CameraItof::adsd3500GetMIPIOutputSpeed(uint16_t &speed) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0034, reinterpret_cast<uint16_t *>(&speed));
}

aditof::Status CameraItof::adsd3500GetImagerErrorCode(uint16_t &errcode) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0038, reinterpret_cast<uint16_t *>(&errcode));
}

aditof::Status CameraItof::adsd3500SetVCSELDelay(uint16_t delay) {
    return m_depthSensor->adsd3500_write_cmd(0x0066, delay);
}

aditof::Status CameraItof::adsd3500GetVCSELDelay(uint16_t &delay) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0068, reinterpret_cast<uint16_t *>(&delay));
}

aditof::Status CameraItof::adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0074, threshold);
}

aditof::Status CameraItof::adsd3500SetJBLFABThreshold(uint16_t threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0075, threshold);
}

aditof::Status CameraItof::adsd3500SetJBLFGaussianSigma(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x006B, value);
}

aditof::Status CameraItof::adsd3500GetJBLFGaussianSigma(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0069, reinterpret_cast<uint16_t *>(&value));
}

aditof::Status CameraItof::adsd3500SetJBLFExponentialTerm(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x006C, value);
}

aditof::Status CameraItof::adsd3500GetJBLFExponentialTerm(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        0x006A, reinterpret_cast<uint16_t *>(&value));
}

aditof::Status CameraItof::adsd3500GetFrameRate(uint16_t &fps) {
    return m_depthSensor->adsd3500_read_cmd(0x0023,
                                            reinterpret_cast<uint16_t *>(&fps));
}

aditof::Status CameraItof::adsd3500SetFrameRate(uint16_t fps) {
    if (fps == 0) {
        fps = 10;
        LOG(WARNING) << "Using a default frame rate of " << fps;
    }

    aditof::Status status = m_depthSensor->setControl("fps", std::to_string(fps));
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set fps at: " << fps << "!";
    } else {
        m_cameraFps = fps;
        LOG(INFO) << "Camera FPS set from Ini file at: " << m_cameraFps;
    }
    return status;
}

aditof::Status CameraItof::adsd3500SetEnableEdgeConfidence(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x0062, value);
}

aditof::Status
CameraItof::adsd3500GetTemperatureCompensationStatus(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0076, reinterpret_cast<uint16_t *>(&value));
}

aditof::Status CameraItof::adsd3500SetEnablePhaseInvalidation(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x0072, value);
}

aditof::Status
CameraItof::adsd3500SetEnableTemperatureCompensation(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x0021, value);
}

aditof::Status CameraItof::adsd3500SetEnableEmbeddedHeaderinAB(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x0036, value);
}

aditof::Status
CameraItof::adsd3500GetEnableEmbeddedHeaderinAB(uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        0x0037, reinterpret_cast<uint16_t *>(&value));
}

aditof::Status CameraItof::adsd3500SetGenericTemplate(uint16_t reg,
                                                      uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(reg, value);
}

aditof::Status CameraItof::adsd3500GetGenericTemplate(uint16_t reg,
                                                      uint16_t &value) {
    return m_depthSensor->adsd3500_read_cmd(
        reg, reinterpret_cast<uint16_t *>(&value));
}

aditof::Status CameraItof::adsd3500GetSensorTemperature(uint16_t &tmpValue) {
    using namespace aditof;
    Status status = Status::OK;

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }
    status = m_depthSensor->adsd3500_read_cmd(0x0054, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(ERROR) << "Can not read sensor temperature";
        return Status::GENERIC_ERROR;
    }

    return status;
}
aditof::Status CameraItof::adsd3500GetLaserTemperature(uint16_t &tmpValue) {
    using namespace aditof;
    Status status = Status::OK;

    unsigned int usDelay = 0;
    if (m_cameraFps > 0) {
        usDelay =
            static_cast<unsigned int>((1 / (double)m_cameraFps) * 1000000);
    }
    status = m_depthSensor->adsd3500_read_cmd(0x0055, &tmpValue, usDelay);
    if (status != Status::OK) {
        LOG(ERROR) << "Can not read laser temperature";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status CameraItof::getKeyValuePairsFromIni(
    const std::string &iniFileName,
    std::map<std::string, std::string> &iniKeyValPairs) {
    using namespace aditof;

    std::ifstream iniStream(iniFileName);
    if (!iniStream.is_open()) {
        LOG(ERROR) << "Failed to open: " << iniFileName;
        return Status::UNREACHABLE;
    }

    iniKeyValPairs.clear();

    std::string line;
    while (getline(iniStream, line)) {
        size_t equalPos = line.find('=');
        if (equalPos == std::string::npos) {
            LOG(WARNING) << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        std::string key = line.substr(0, equalPos);
        std::string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            m_iniKeyValPairs.emplace(key, value);
        } else {
            LOG(WARNING) << "No value found for parameter: " << key;
        }
    }

    iniStream.close();

    return Status::OK;
}

void CameraItof::setAdsd3500WithIniParams(
    const std::map<std::string, std::string> &iniKeyValPairs) {

    auto it = iniKeyValPairs.find("abThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetABinvalidationThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "abThreshMin was not found in .ini file";
    }

    it = iniKeyValPairs.find("confThresh");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetConfidenceThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "confThresh was not found in .ini file";
    }

    it = iniKeyValPairs.find("radialThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetRadialThresholdMin(std::stoi(it->second));
    } else {
        LOG(WARNING) << "radialThreshMin was not found in .ini file";
    }

    it = iniKeyValPairs.find("radialThreshMax");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetRadialThresholdMax(std::stoi(it->second));
    } else {
        LOG(WARNING) << "radialThreshMax was not found in .ini file";
    }

    it = iniKeyValPairs.find("jblfWindowSize");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFfilterSize(std::stoi(it->second));
    } else {
        LOG(WARNING) << "jblfWindowSize was not found in .ini file";
    }

    it = iniKeyValPairs.find("jblfApplyFlag");
    if (it != iniKeyValPairs.end()) {
        bool en = !(it->second == "0");
        adsd3500SetJBLFfilterEnableState(en);
    } else {
        LOG(WARNING) << "jblfApplyFlag was not found in .ini file";
    }

    it = iniKeyValPairs.find("fps");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetFrameRate(std::stoi(it->second));
    } else {
        LOG(WARNING) << "fps was not found in .ini file";
    }
}
