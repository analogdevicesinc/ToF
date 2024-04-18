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
#include "utils_ini.h"

#include "cJSON.h"
#include "crc.h"
#include "tofi/algorithms.h"
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
#include <math.h>
#include <string>
#include <thread>
#include <vector>

static const int skMetaDataBytesCount = 128;

CameraItof::CameraItof(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    const std::string &ubootVersion, const std::string &kernelVersion,
    const std::string &sdCardImageVersion, const std::string &netLinkTest)
    : m_depthSensor(depthSensor), m_devStarted(false), m_adsd3500Enabled(false),
      m_loadedConfigData(false), m_xyzEnabled(false), m_xyzSetViaApi(false),
      m_cameraFps(0), m_fsyncMode(-1), m_mipiOutputSpeed(-1),
      m_enableTempCompenstation(-1), m_enableMetaDatainAB(-1),
      m_enableEdgeConfidence(-1), m_modesVersion(0),
      m_xyzTable({nullptr, nullptr, nullptr}),
      m_imagerType(aditof::ImagerType::UNSET), m_dropFirstFrame(true),
      m_dropFrameOnce(true) {

    FloatToLinGenerateTable();
    memset(&m_xyzTable, 0, sizeof(m_xyzTable));
    m_details.mode = "sr-native";
    m_details.uBootVersion = ubootVersion;
    m_details.kernelVersion = kernelVersion;
    m_details.sdCardImageVersion = sdCardImageVersion;
    m_netLinkTest = netLinkTest;

    // Define some of the controls of this camera
    // For now there are none. To add one use: m_controls.emplace("your_control_name", "default_control_value");
    // And handle the control action in setControl()

    // Check Depth Sensor
    if (!depthSensor) {
        LOG(WARNING) << "Invalid instance of a depth sensor";
        return;
    }

    aditof::SensorDetails sDetails;
    m_depthSensor->getDetails(sDetails);
    m_details.cameraId = sDetails.id;
    m_details.connection = sDetails.connectionType;

    std::string sensorName;
    m_depthSensor->getName(sensorName);
    LOG(INFO) << "Sensor name = " << sensorName;
    if (sensorName == "adsd3500") {
        m_adsd3500Enabled = true;
    } else if (sensorName == "offline") {
        m_isOffline = true;
    }

    m_adsd3500_master = true;
}

CameraItof::~CameraItof() {
    freeConfigData();
    // m_device->toggleFsync();
    cleanupXYZtables();
}

aditof::Status CameraItof::initialize(const std::string &configFilepath) {
    using namespace aditof;
    Status status = Status::OK;

    LOG(INFO) << "Initializing camera";

    if (!m_adsd3500Enabled && !m_isOffline) {
        LOG(ERROR) << "This usecase is no longer supported.";
        return aditof::Status::UNAVAILABLE;
    }

    m_initConfigFilePath = configFilepath;

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

    if (!m_netLinkTest.empty()) {
        m_depthSensor->setControl("netlinktest", "1");
    }

    // get imager type that is used toghether with ADSD3500
    std::string controlValue;
    status = m_depthSensor->getControl("imagerType", controlValue);
    if (status == Status::OK) {
        if (controlValue == "1") {
            m_imagerType = ImagerType::ADSD3100;
        } else if (controlValue == "2") {
            m_imagerType = ImagerType::ADSD3030;
        } else {
            m_imagerType = ImagerType::UNSET;
            LOG(ERROR) << "Unkown imager type: " << controlValue;
            return Status::UNAVAILABLE;
        }

        status = m_depthSensor->getAvailableModes(m_availableModesName);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to get available frame types name!";
            return status;
        }

        for (auto availablemodes : m_availableModesName) {
            status =
                m_depthSensor->getModeDetails(availablemodes, m_frameDetails);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to get available frame types details!";
                return status;
            }
            m_availableSensorFrameTypes.emplace_back(m_frameDetails);
            uint8_t intrinsics[56] = {0};
            uint8_t dealiasParams[32] = {0};
            TofiXYZDealiasData dealiasStruct;
            //the first element of readback_data for adsd3500_read_payload is used for the custom command
            //it will be overwritten by the returned data
            uint8_t mode = m_frameDetails.modeNumber;

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

    //Populate the data from the json file provided
    status = parseJsonFileContent();
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to parse Json file!";
        return status;
    }

    aditof::Status configStatus = loadConfigData();
    if (configStatus == aditof::Status::OK) {
        m_loadedConfigData = true;
    } else {
        LOG(ERROR) << "loadConfigData failed";
        return Status::GENERIC_ERROR;
    }

    if (m_fsyncMode >= 0) {
        status = adsd3500SetToggleMode(m_fsyncMode);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set fsyncMode.";
            return status;
        }
    } else {
        LOG(WARNING) << "fsyncMode is not being set by SDK.";
    }

    if (m_mipiOutputSpeed >= 0) {
        status = adsd3500SetMIPIOutputSpeed(m_mipiOutputSpeed);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set mipiOutputSpeed.";
            return status;
        }
    } else {
        LOG(WARNING) << "mipiSpeed is not being set by SDK.";
    }

    if (m_enableTempCompenstation >= 0) {
        status =
            adsd3500SetEnableTemperatureCompensation(m_enableTempCompenstation);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set enableTempCompenstation.";
            return status;
        }
    } else {
        LOG(WARNING) << "enableTempCompenstation is not being set by SDK.";
    }

    if (m_enableEdgeConfidence >= 0) {
        status = adsd3500SetEnableEdgeConfidence(m_enableEdgeConfidence);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set enableEdgeConfidence.";
            return status;
        }
    } else {
        LOG(WARNING) << "enableEdgeConfidence is not being set by SDK.";
    }

    std::string serialNumber;
    status = readSerialNumber(serialNumber);
    if (status == Status::OK) {
        LOG(INFO) << "Module serial number: " << serialNumber;
    } else if (status == Status::UNAVAILABLE) {
        LOG(INFO) << "Serial read is not supported in this firmware!";
    } else {
        LOG(ERROR) << "Failed to read serial number!";
        return status;
    }

    LOG(INFO) << "Camera initialized";

    return Status::OK;
}

aditof::Status CameraItof::start() {
    using namespace aditof;

    Status status = m_depthSensor->start();
    if (Status::OK != status) {
        LOG(ERROR) << "Error starting adsd3500.";
        return status;
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

aditof::Status CameraItof::setMode(const std::string &mode) {
    using namespace aditof;
    Status status = Status::OK;

    auto modeIt = std::find_if(
        m_availableSensorFrameTypes.begin(), m_availableSensorFrameTypes.end(),
        [&mode](const DepthSensorFrameType &d) { return (d.mode == mode); });

    if (modeIt == m_availableSensorFrameTypes.end()) {
        LOG(WARNING) << "Mode: " << mode << " not supported by camera";
        return Status::INVALID_ARGUMENT;
    }

    if (m_ini_depth_map.size() > 1) {
        m_ini_depth = m_ini_depth_map[mode];
    }

    if (m_details.connection == ConnectionType::USB) {
        status = m_depthSensor->adsd3500_reset();
        if (status != Status::OK) {
            LOG(WARNING) << "Failed to reset the camera!";
            return status;
        }
    }

    if (mode == "pcm-native") {
        m_pcmFrame = true;
    } else {
        m_pcmFrame = false;
    }

    UtilsIni::getKeyValuePairsFromIni(m_ini_depth, m_iniKeyValPairs);
    setAdsd3500WithIniParams(m_iniKeyValPairs);
    configureSensorFrameType();
    m_details.mode = mode;

    if (m_enableMetaDatainAB > 0) {
        if (!m_pcmFrame) {
            status = adsd3500SetEnableMetadatainAB(m_enableMetaDatainAB);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to set enableMetaDatainAB.";
                return status;
            }
            LOG(INFO) << "Metadata in AB is enabled and it is stored in the "
                         "first 128 bytes.";

        } else {
            status = adsd3500SetEnableMetadatainAB(0);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to disable enableMetaDatainAB.";
                return status;
            }
            LOG(INFO) << "Metadata in AB is disabled for this frame type.";
        }

    } else {
        status = adsd3500SetEnableMetadatainAB(m_enableMetaDatainAB);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set enableMetaDatainAB.";
            return status;
        }

        LOG(WARNING) << "Metadata in AB is disabled.";
    }

    LOG(INFO) << "Using ini file: " << m_ini_depth;

    status = m_depthSensor->setMode(*modeIt);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to set frame type";
        return status;
    }

    status = m_depthSensor->getModeDetails(mode, m_frameDetails);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get frame type details!";
        return status;
    }

    // Store the frame details in camera details
    m_details.frameType.type = (*modeIt).mode;
    m_details.frameType.width = (*modeIt).baseResolutionWidth;
    m_details.frameType.height = (*modeIt).baseResolutionHeight;
    m_details.frameType.totalCaptures = 1;
    m_details.frameType.dataDetails.clear();
    for (const auto &item : (*modeIt).frameContent) {
        if (item == "xyz" && !m_xyzEnabled) {
            continue;
        }
        if (item == "raw") { // "raw" is not supported right now
            continue;
        }

        FrameDataDetails fDataDetails;
        fDataDetails.type = item;
        fDataDetails.width = m_frameDetails.baseResolutionWidth;
        fDataDetails.height = m_frameDetails.baseResolutionHeight;
        fDataDetails.subelementSize = sizeof(uint16_t);
        fDataDetails.subelementsPerElement = 1;

        if (item == "xyz") {
            fDataDetails.subelementsPerElement = 3;
        } else if (item == "raw") {
            fDataDetails.width = m_frameDetails.frameWidthInBytes;
            fDataDetails.height = m_frameDetails.frameHeightInBytes;
            fDataDetails.subelementSize = 1;
            fDataDetails.subelementsPerElement = 1;
        } else if (item == "metadata") {
            fDataDetails.subelementSize = 1;
            fDataDetails.width = 128;
            fDataDetails.height = 1;
        } else if (item == "conf") {
            fDataDetails.subelementSize = sizeof(float);
        }
        fDataDetails.bytesCount = fDataDetails.width * fDataDetails.height *
                                  fDataDetails.subelementSize *
                                  fDataDetails.subelementsPerElement;

        m_details.frameType.dataDetails.emplace_back(fDataDetails);
    }

    // We want computed frames (Depth & AB). Tell target to initialize depth compute
    if (!m_pcmFrame) {
        if (!m_ini_depth.empty()) {
            size_t dataSize = m_depthINIData.size;
            unsigned char *pData = m_depthINIData.p_data;

            if (m_depthINIDataMap.size() > 1) {
                dataSize = m_depthINIDataMap[m_ini_depth].size;
                pData = m_depthINIDataMap[m_ini_depth].p_data;
            }

            // Disable the generation of XYZ frames on target
            std::string s(reinterpret_cast<const char *>(pData), dataSize);
            std::string::size_type n;
            n = s.find("xyzEnable=");
            if (n == std::string::npos) {
                DLOG(INFO) << "xyzEnable not found in .ini. Can't set it to 0.";
            } else {
                s[n + strlen("xyzEnable=")] = '0';
            }

            aditof::Status localStatus;
            localStatus = m_depthSensor->initTargetDepthCompute(
                (uint8_t *)s.c_str(), dataSize, (uint8_t *)m_xyz_dealias_data,
                sizeof(TofiXYZDealiasData) * 10);
            if (localStatus != aditof::Status::OK) {
                LOG(ERROR) << "Failed to initialize depth compute on target!";
                return localStatus;
            }

            std::string depthComputeStatus;
            localStatus = m_depthSensor->getControl("depthComputeOpenSource",
                                                    depthComputeStatus);
            if (localStatus == aditof::Status::OK) {
                if (depthComputeStatus == "0") {
                    LOG(INFO) << "Using closed source depth compute library.";
                } else {
                    LOG(INFO) << "Using open source depth compute library.";
                }
            } else {
                LOG(ERROR)
                    << "Failed to get depth compute version from target!";
            }
        }
    }

    // If we compute XYZ then prepare the XYZ tables which depend on the mode
    if (m_xyzEnabled && !m_pcmFrame) {
        uint8_t mode = m_frameDetails.modeNumber;

        const int GEN_XYZ_ITERATIONS = 20;
        TofiXYZDealiasData *pDealias = &m_xyz_dealias_data[mode];

        cleanupXYZtables();
        int ret = Algorithms::GenerateXYZTables(
            &m_xyzTable.p_x_table, &m_xyzTable.p_y_table, &m_xyzTable.p_z_table,
            &(pDealias->camera_intrinsics), pDealias->n_sensor_rows,
            pDealias->n_sensor_cols, m_frameDetails.frameWidthInBytes,
            m_frameDetails.frameHeightInBytes, pDealias->n_offset_rows,
            pDealias->n_offset_cols, pDealias->row_bin_factor,
            pDealias->col_bin_factor, GEN_XYZ_ITERATIONS);
        if (ret != 0 || !m_xyzTable.p_x_table || !m_xyzTable.p_y_table ||
            !m_xyzTable.p_z_table) {
            LOG(ERROR) << "Failed to generate the XYZ tables";
            return Status::GENERIC_ERROR;
        }
    }

    return status;
}

aditof::Status CameraItof::getIniParams(std::map<std::string, float> &params) {
    aditof::Status status;
    status = m_depthSensor->getIniParams(params);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "get ini parameters failed.";
    }
    return status;
}

aditof::Status CameraItof::setIniParams(std::map<std::string, float> &params) {
    aditof::Status status;

    status = adsd3500SetIniParams(params);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "set ini parameters failed in chip.";
    }
    status = m_depthSensor->setIniParams(params);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "set ini parameters failed in depth-compute.";
    }
    return status;
}

aditof::Status
CameraItof::getAvailableModes(std::vector<std::string> &availableModes) const {
    using namespace aditof;
    Status status = Status::OK;
    availableModes.clear();

    for (const auto &mode : m_availableModesName) {
        availableModes.emplace_back(mode);
    }

    return status;
}

aditof::Status CameraItof::requestFrame(aditof::Frame *frame) {
    using namespace aditof;
    Status status = Status::OK;

    if (frame == nullptr) {
        return Status::INVALID_ARGUMENT;
    }

    FrameDetails frameDetails;
    frame->getDetails(frameDetails);

    if (m_details.frameType != frameDetails) {
        frame->setDetails(m_details.frameType);
    }

    uint16_t *frameDataLocation = nullptr;
    if (!m_pcmFrame) {
        frame->getData("frameData", &frameDataLocation);
    } else {
        if ((m_details.frameType.type == "pcm-native")) {
            frame->getData("ab", &frameDataLocation);
        } else if (m_details.frameType.type == "") {
            LOG(ERROR) << "Frame type not found!";
            return Status::INVALID_ARGUMENT;
        }
    }
    if (!frameDataLocation) {
        LOG(WARNING) << "getframe failed to allocated valid frame";
        return status;
    }

    if (m_dropFirstFrame && m_dropFrameOnce) {
        m_depthSensor->getFrame(frameDataLocation);
        m_dropFrameOnce = false;
        LOG(INFO) << "Dropped first frame";
    }

    status = m_depthSensor->getFrame(frameDataLocation);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get frame from device";
        return status;
    }

    // The incoming sensor frames are already processed. Need to just create XYZ data
    if (m_xyzEnabled) {
        uint16_t *depthFrame;
        uint16_t *xyzFrame;

        frame->getData("depth", &depthFrame);
        frame->getData("xyz", &xyzFrame);

        Algorithms::ComputeXYZ((const uint16_t *)depthFrame, &m_xyzTable,
                               (int16_t *)xyzFrame,
                               m_frameDetails.baseResolutionHeight,
                               m_frameDetails.baseResolutionWidth);
    }

    Metadata metadata;
    status = frame->getMetadataStruct(metadata);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame metadata!";
        return status;
    }

    uint16_t *metadataLocation;
    status = frame->getData("metadata", &metadataLocation);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get metadata location";
        return status;
    }

    if (m_enableMetaDatainAB) {
        uint16_t *abFrame;
        frame->getData("ab", &abFrame);
        memcpy(reinterpret_cast<uint8_t *>(&metadata), abFrame,
               sizeof(metadata));
    } else {
        // If metadata from ADSD3500 is not available/disabled, generate one here
        memset(static_cast<void *>(&metadata), 0, sizeof(metadata));
        metadata.width = m_frameDetails.baseResolutionWidth;
        metadata.height = m_frameDetails.baseResolutionHeight;
        metadata.imagerMode = m_frameDetails.modeNumber;
        metadata.bitsInDepth = m_depthBitsPerPixel;
        metadata.bitsInAb = m_abBitsPerPixel;
        metadata.bitsInConfidence = m_confBitsPerPixel;

        // For frame with PCM content we need to store ab bits
        if (m_pcmFrame) {
            metadata.bitsInAb = 16;
        }
    }

    metadata.xyzEnabled = m_xyzEnabled;
    memcpy(reinterpret_cast<uint8_t *>(metadataLocation),
           reinterpret_cast<uint8_t *>(&metadata), sizeof(metadata));

    return Status::OK;
}

aditof::Status CameraItof::normalizeABdata(aditof::Frame *frame,
                                           bool useLogScaling) {

    using namespace aditof;
    Status status = Status::OK;
    uint16_t *abVideoData;

    status = frame->getData("ab", &abVideoData);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
        return status;
    }

    if (!abVideoData) {
        LOG(ERROR) << "no memory allocated in frame";
        return Status::INVALID_ARGUMENT;
    }

    aditof::FrameDataDetails frameAbDetails;
    frameAbDetails.height = 0;
    frameAbDetails.width = 0;
    frame->getDataDetails("ab", frameAbDetails);

    size_t imageSize = frameAbDetails.height * frameAbDetails.width;

    uint32_t min_value_of_AB_pixel = 0xFFFF;
    uint32_t max_value_of_AB_pixel = 1;

    for (size_t dummyCtr = 0; dummyCtr < imageSize; ++dummyCtr) {
        if (abVideoData[dummyCtr] > max_value_of_AB_pixel) {
            max_value_of_AB_pixel = abVideoData[dummyCtr];
        }
        if (abVideoData[dummyCtr] < min_value_of_AB_pixel) {
            min_value_of_AB_pixel = abVideoData[dummyCtr];
        }
    }
    max_value_of_AB_pixel -= min_value_of_AB_pixel;

    double c = 255.0f / log10(1 + max_value_of_AB_pixel);

    for (size_t dummyCtr = 0; dummyCtr < imageSize; ++dummyCtr) {

        abVideoData[dummyCtr] -= min_value_of_AB_pixel;

        double pix = abVideoData[dummyCtr] * (255.0 / max_value_of_AB_pixel);

        pix = (pix >= 255.0) ? 255.0 : pix;

        if (useLogScaling) {
            pix = c * log10(pix + 1);
        }
        abVideoData[dummyCtr] = (uint8_t)(pix);
    }

    return status;
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

    if (m_depthINIDataMap.size() > 0) {
        for (auto it = m_depthINIDataMap.begin(); it != m_depthINIDataMap.end();
             ++it) {
            free((void *)(it->second.p_data));
        }
        m_depthINIDataMap.clear();
    }
    if (m_calData.p_data) {
        free((void *)(m_calData.p_data));
        m_calData.p_data = nullptr;
        m_calData.size = 0;
    }
}

aditof::Status CameraItof::readSerialNumber(std::string &serialNumber,
                                            bool useCacheValue) {
    using namespace aditof;
    Status status = Status::OK;

    if (m_adsd3500FwVersionInt < 4710) {
        LOG(WARNING) << "Serial read is not supported in this firmware!";
        return Status::UNAVAILABLE;
    }

    if (useCacheValue) {
        if (!m_details.serialNumber.empty()) {
            serialNumber = m_details.serialNumber;
            return status;
        } else {
            LOG(INFO)
                << "No serial number stored in cache. Reading from memory.";
        }
    }

    uint8_t serial[32] = {0};

    status = m_depthSensor->adsd3500_read_payload_cmd(0x19, serial, 32);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read serial number!";
        return status;
    }

    m_details.serialNumber = std::string(reinterpret_cast<char *>(serial), 32);
    serialNumber = m_details.serialNumber;

    return status;
}

aditof::Status CameraItof::saveModuleCCB(const std::string &filepath) {
    if (filepath.empty()) {
        LOG(ERROR) << "File path where CCB should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    std::string ccbContent;
    aditof::Status status = readAdsd3500CCB(ccbContent);

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read CCB from adsd3500 module!";
        return aditof::Status::GENERIC_ERROR;
    }

    std::ofstream destination(filepath, std::ios::binary);
    destination << ccbContent;

    return aditof::Status::OK;
}

aditof::Status CameraItof::saveModuleCFG(const std::string &filepath) const {
    if (filepath.empty()) {
        LOG(ERROR) << "File path where CFG should be written is empty.";
        return aditof::Status::INVALID_ARGUMENT;
    }

    LOG(ERROR) << "CFG files is unavailable";
    return aditof::Status::UNAVAILABLE;
}

aditof::Status CameraItof::enableXYZframe(bool enable) {
    m_xyzEnabled = enable;
    m_xyzSetViaApi = true;

    return aditof::Status::OK;
}

aditof::Status CameraItof::enableDepthCompute(bool enable) {
    return aditof::Status::UNAVAILABLE;
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

aditof::Status
CameraItof::adsd3500UpdateFirmware(const std::string &fwFilePath) {
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
    std::ifstream fw_file(fwFilePath, std::ios::binary);
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
            if (j < static_cast<int>(fw_len)) {
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
        m_depthSensor->adsd3500_unregister_interrupt_callback(cb);
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

aditof::Status CameraItof::readAdsd3500CCB(std::string &ccb) {
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
    ccb = std::string((char *)ccbContent, ccbFileSize - 4);

    delete[] ccbContent;

    return status;
}

void CameraItof::configureSensorFrameType() {
    std::string value;

    auto it = m_iniKeyValPairs.find("bitsInPhaseOrDepth");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        m_depthBitsPerPixel = std::stoi(value);
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
        m_confBitsPerPixel = std::stoi(value);
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

    it = m_iniKeyValPairs.find("inputFormat");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        m_depthSensor->setControl("inputFormat", value);
    } else {
        LOG(WARNING) << "inputFormat was not found in .ini file";
    }

    // XYZ set through camera control takes precedence over the setting from .ini file
    if (!m_xyzSetViaApi) {
        it = m_iniKeyValPairs.find("xyzEnable");
        if (it != m_iniKeyValPairs.end()) {
            m_xyzEnabled = !(it->second == "0");
        } else {
            LOG(WARNING) << "xyzEnable was not found in .ini file";
        }
    }

    //Embedded header is being set from the ini file
    it = m_iniKeyValPairs.find("headerSize");
    if (it != m_iniKeyValPairs.end()) {
        value = it->second;
        if (std::stoi(value) == 128) {
            m_enableMetaDatainAB = 1;
        } else {
            m_enableMetaDatainAB = 0;
        }
    } else {
        LOG(WARNING) << "headerSize was not found in .ini file";
    }
}

static int16_t getValueFromJSON(cJSON *config_json, std::string key) {
    int16_t value = -1;
    const cJSON *json_value =
        cJSON_GetObjectItemCaseSensitive(config_json, key.c_str());
    if (cJSON_IsString(json_value) && (json_value->valuestring != NULL)) {
        value = atoi(json_value->valuestring);
    }
    return value;
}

aditof::Status CameraItof::parseJsonFileContent() {
    using namespace aditof;
    Status status = Status::OK;

    // Parse config.json
    std::string config = m_initConfigFilePath;
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

        // Get depth ini file location
        const cJSON *json_depth_ini_file = nullptr;
        json_depth_ini_file =
            cJSON_GetObjectItemCaseSensitive(config_json, "depthIni");
        if (cJSON_IsString(json_depth_ini_file) == false) { // Check for old key
            json_depth_ini_file =
                cJSON_GetObjectItemCaseSensitive(config_json, "DEPTH_INI");
        }
        if (cJSON_IsString(json_depth_ini_file) &&
            (json_depth_ini_file->valuestring != NULL)) {
            // store depth ini file location
            std::string mode;
            std::vector<std::string> iniFiles;

            m_ini_depth_map.clear();
            m_ini_depth.clear();

            Utils::splitIntoTokens(
                std::string(json_depth_ini_file->valuestring), ';', iniFiles);
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

        m_fsyncMode = getValueFromJSON(config_json, "fsyncMode"); // New key
        if (m_fsyncMode < 0) { // Check for old key
            m_fsyncMode = getValueFromJSON(config_json, "FSYNC_MODE");
        }
        m_mipiOutputSpeed = getValueFromJSON(config_json, "mipiSpeed");
        m_enableTempCompenstation =
            getValueFromJSON(config_json, "enableTempCompenstation");
        m_enableEdgeConfidence =
            getValueFromJSON(config_json, "enableEdgeConfidence");

        // Delete memory allocated for JSON
        cJSON_Delete(config_json);

    } else if (!config.empty()) {
        LOG(ERROR) << "Couldn't parse config file: " << config.c_str();
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status
CameraItof::saveDepthParamsToJsonFile(const std::string &savePathFile) {
    using namespace aditof;
    Status status = Status::OK;

    cJSON *rootjson = cJSON_CreateObject();
    cJSON *depthParamjson = cJSON_CreateObject();
    cJSON_AddItemToObject(rootjson, "depthParams", depthParamjson);

    for (auto pfile = m_ini_depth_map.begin(); pfile != m_ini_depth_map.end();
         pfile++) {

        std::map<std::string, std::string> iniKeyValPairs;
        status =
            UtilsIni::getKeyValuePairsFromIni(pfile->second, iniKeyValPairs);

        iniKeyValPairs.emplace("errata1",
                               std::to_string(m_dropFirstFrame ? 1 : 0));

        if (status == Status::OK) {
            cJSON *json = cJSON_CreateObject();
            for (auto item = iniKeyValPairs.begin();
                 item != iniKeyValPairs.end(); item++) {
                double valued = strtod(item->second.c_str(), NULL);
                if (isConvertibleToDouble(item->second)) {
                    cJSON_AddNumberToObject(json, item->first.c_str(), valued);
                } else {
                    cJSON_AddStringToObject(json, item->first.c_str(),
                                            item->second.c_str());
                }
            }
            cJSON_AddItemToObject(depthParamjson, pfile->first.c_str(), json);
        }
    }

    char *json_str = cJSON_Print(rootjson);

    FILE *fp = fopen(savePathFile.c_str(), "w");
    if (fp == NULL) {
        LOG(WARNING) << " Unable to open the file. " << savePathFile.c_str();
        return Status::GENERIC_ERROR;
    }
    fputs(json_str, fp);
    fclose(fp);

    cJSON_free(json_str);
    cJSON_Delete(rootjson);

    return status;
}

aditof::Status
CameraItof::loadDepthParamsFromJsonFile(const std::string &pathFile,
                                        const std::string &frameType) {

    using namespace aditof;
    Status status = Status::OK;

    // Parse json
    std::ifstream ifs(pathFile.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    cJSON *config_json = cJSON_Parse(content.c_str());
    if (config_json != NULL) {

        const cJSON *depthParams =
            cJSON_GetObjectItemCaseSensitive(config_json, "depthParams");

        if (depthParams) {

            cJSON *depthframeType = cJSON_GetObjectItemCaseSensitive(
                depthParams, frameType.c_str());

            if (depthframeType) {

                std::map<std::string, std::string> iniKeyValPairs;

                cJSON *elem;
                cJSON_ArrayForEach(elem, depthframeType) {

                    std::string value = "";
                    if (elem->valuestring != nullptr) {
                        value = std::string(elem->valuestring);
                    } else {
                        value = std::to_string(elem->valuedouble);
                    }
                    if (std::string(elem->string) == "errata1") {
                        if (elem->valuedouble == 1) {
                            m_dropFirstFrame = true;
                        } else {
                            m_dropFirstFrame = false;
                        }
                    }
                    iniKeyValPairs.emplace(std::string(elem->string), value);
                    LOG(INFO)
                        << "Found key value: " << std::string(elem->string)
                        << " - " << value;
                }
                setAdsd3500WithIniParams(iniKeyValPairs);
            }
        }
    }
    return status;
}

bool CameraItof::isConvertibleToDouble(const std::string &str) {
    bool result = false;
    try {
        std::stod(str);
        result = true;
    } catch (...) {
    }
    return result;
}

void CameraItof::dropFirstFrame(bool dropFrame) {
    m_dropFirstFrame = dropFrame;
}

aditof::Status
CameraItof::setSensorConfiguration(const std::string &sensorConf) {
    return m_depthSensor->setSensorConfiguration(sensorConf);
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

    m_adsd3500FwVersionInt = 0;
    for (int i = 0; i < 4; i++) {
        m_adsd3500FwVersionInt = m_adsd3500FwVersionInt * 10 + fwData[i];
    }

    fwVersion = m_adsd3500FwGitHash.first;
    fwHash = m_adsd3500FwGitHash.second;

    return status;
}

aditof::Status CameraItof::adsd3500SetABinvalidationThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0010, threshold);
}

aditof::Status CameraItof::adsd3500GetABinvalidationThreshold(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        0x0015, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetConfidenceThreshold(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0011, threshold);
}
aditof::Status CameraItof::adsd3500GetConfidenceThreshold(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        0x0016, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetJBLFfilterEnableState(bool enable) {
    return m_depthSensor->adsd3500_write_cmd(0x0013, enable ? 1 : 0);
}
aditof::Status CameraItof::adsd3500GetJBLFfilterEnableState(bool &enabled) {
    int intEnabled = 0;
    aditof::Status status = m_depthSensor->adsd3500_read_cmd(
        0x0017, reinterpret_cast<uint16_t *>(&intEnabled));
    enabled = !!intEnabled;
    return status;
}

aditof::Status CameraItof::adsd3500SetJBLFfilterSize(int size) {
    return m_depthSensor->adsd3500_write_cmd(0x0014, size);
}
aditof::Status CameraItof::adsd3500GetJBLFfilterSize(int &size) {
    size = 0;
    return m_depthSensor->adsd3500_read_cmd(
        0x0018, reinterpret_cast<uint16_t *>(&size));
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMin(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0027, threshold);
}
aditof::Status CameraItof::adsd3500GetRadialThresholdMin(int &threshold) {
    threshold = 0;
    return m_depthSensor->adsd3500_read_cmd(
        0x0028, reinterpret_cast<uint16_t *>(&threshold));
}

aditof::Status CameraItof::adsd3500SetRadialThresholdMax(int threshold) {
    return m_depthSensor->adsd3500_write_cmd(0x0029, threshold);
}

aditof::Status CameraItof::adsd3500GetRadialThresholdMax(int &threshold) {
    threshold = 0;
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
    value = 0;
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

    aditof::Status status =
        m_depthSensor->setControl("fps", std::to_string(fps));
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

aditof::Status CameraItof::adsd3500SetEnableMetadatainAB(uint16_t value) {
    return m_depthSensor->adsd3500_write_cmd(0x0036, value);
}

aditof::Status CameraItof::adsd3500GetEnableMetadatainAB(uint16_t &value) {
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

aditof::Status CameraItof::adsd3500GetStatus(int &chipStatus,
                                             int &imagerStatus) {
    using namespace aditof;
    Status status = Status::OK;
    status = m_depthSensor->adsd3500_get_status(chipStatus, imagerStatus);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to read chip/imager status!";
        return status;
    }

    if (chipStatus != 0) {
        LOG(ERROR) << "ADSD3500 error detected: "
                   << m_adsdErrors.GetStringADSD3500(chipStatus);

        if (chipStatus == m_adsdErrors.ADSD3500_STATUS_IMAGER_ERROR) {
            if (m_imagerType == ImagerType::ADSD3100) {
                LOG(ERROR) << "ADSD3100 imager error detected: "
                           << m_adsdErrors.GetStringADSD3100(imagerStatus);
            } else if (m_imagerType == ImagerType::ADSD3030) {
                LOG(ERROR) << "ADSD3030 imager error detected: "
                           << m_adsdErrors.GetStringADSD3030(imagerStatus);
            } else {
                LOG(ERROR) << "Imager error detected. Cannot be displayed "
                              "because imager type is unknown";
            }
        }
    } else {
        LOG(INFO) << "No chip/imager errors detected.";
    }

    return status;
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

void CameraItof::setAdsd3500WithIniParams(
    const std::map<std::string, std::string> &iniKeyValPairs) {

    auto it = iniKeyValPairs.find("abThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetABinvalidationThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "abThreshMin was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("confThresh");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetConfidenceThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "confThresh was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("radialThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetRadialThresholdMin(std::stoi(it->second));
    } else {
        LOG(WARNING)
            << "radialThreshMin was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("radialThreshMax");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetRadialThresholdMax(std::stoi(it->second));
    } else {
        LOG(WARNING)
            << "radialThreshMax was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("jblfWindowSize");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFfilterSize(std::stoi(it->second));
    } else {
        LOG(WARNING)
            << "jblfWindowSize was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("jblfApplyFlag");
    if (it != iniKeyValPairs.end()) {
        bool en = !(it->second == "0");
        adsd3500SetJBLFfilterEnableState(en);
    } else {
        LOG(WARNING)
            << "jblfApplyFlag was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("fps");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetFrameRate(std::stoi(it->second));
    } else {
        LOG(WARNING) << "fps was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("vcselDelay");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetVCSELDelay(std::stoi(it->second));
    } else {
        LOG(WARNING) << "vcselDelay was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("jblfMaxEdge");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFMaxEdgeThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "jblfMaxEdge was not found in .ini file, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblfABThreshold");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFABThreshold(std::stoi(it->second));
    } else {
        LOG(WARNING) << "jblfABThreshold was not found in .ini file";
    }

    it = iniKeyValPairs.find("jblfGaussianSigma");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFGaussianSigma(std::stoi(it->second));
    } else {
        LOG(WARNING)
            << "jblfGaussianSigma was not found in .ini file, not setting.";
    }

    it = iniKeyValPairs.find("jblfExponentialTerm");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetJBLFExponentialTerm(std::stoi(it->second));
    } else {
        LOG(WARNING) << "jblfExponentialTerm was not found in .ini file, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("enablePhaseInvalidation");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetEnablePhaseInvalidation(std::stoi(it->second));
    } else {
        LOG(WARNING) << "enablePhaseInvalidation was not found in .ini file, "
                        "not setting.";
    }
}

aditof::Status CameraItof::adsd3500SetIniParams(
    const std::map<std::string, float> &iniKeyValPairs) {
    using namespace aditof;
    Status status = Status::OK;

    auto it = iniKeyValPairs.find("ab_thresh_min");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetABinvalidationThreshold(int(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "ab_thresh_min was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("conf_thresh");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetConfidenceThreshold(int(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "conf_thresh was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("radial_thresh_min");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetRadialThresholdMin(int(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "radial_thresh_min was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("radial_thresh_max");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetRadialThresholdMax(int(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "radial_thresh_max was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("jblf_window_size");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFfilterSize(int(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "jblf_window_size was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("jblf_apply_flag");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFfilterEnableState(it->second);
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "jblf_apply_flag was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("fps");
    if (it != iniKeyValPairs.end()) {
        adsd3500SetFrameRate(it->second);
    } else {
        LOG(WARNING) << "fps was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("vcselDelay");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetVCSELDelay(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "vcselDelay was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("jblf_max_edge");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFMaxEdgeThreshold(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "jblf_max_edge was not found in ini params, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("jblf_ab_threshold");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFABThreshold(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "jblf_ab_threshold was not found in ini params";
    }

    it = iniKeyValPairs.find("jblf_gaussian_sigma");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFGaussianSigma(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING)
            << "jblf_gaussian_sigma was not found in ini params, not setting.";
    }

    it = iniKeyValPairs.find("jblf_exponential_term");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetJBLFExponentialTerm(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "jblf_exponential_term was not found in ini paramse, "
                        "not setting.";
    }

    it = iniKeyValPairs.find("enablePhaseInvalidation");
    if (it != iniKeyValPairs.end()) {
        status = adsd3500SetEnablePhaseInvalidation(uint16_t(it->second));
        if (status != Status::OK) {
            LOG(ERROR) << "Request to write registers failed";
            return status;
        }
    } else {
        LOG(WARNING) << "enablePhaseInvalidation was not found in ini params, "
                        "not setting.";
    }
    return status;
}

void CameraItof::cleanupXYZtables() {
    if (m_xyzTable.p_x_table) {
        free((void *)m_xyzTable.p_x_table);
        m_xyzTable.p_x_table = nullptr;
    }
    if (m_xyzTable.p_y_table) {
        free((void *)m_xyzTable.p_y_table);
        m_xyzTable.p_y_table = nullptr;
    }
    if (m_xyzTable.p_z_table) {
        free((void *)m_xyzTable.p_z_table);
        m_xyzTable.p_z_table = nullptr;
    }
}

aditof::Status CameraItof::getImagerType(aditof::ImagerType &imagerType) const {
    imagerType = m_imagerType;

    return aditof::Status::OK;
}
