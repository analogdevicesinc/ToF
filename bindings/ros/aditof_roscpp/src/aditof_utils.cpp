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
#include "aditof_utils.h"

#include <aditof/frame.h>
#include <aditof/system.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string.h>
#include <unistd.h>

std::mutex mtx_dynamic_rec;
using namespace aditof;

std::string parseArgs(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    if (argc > 1) {
        std::string ip = argv[1];
        if (!ip.empty()) {
            return ip;
        }
    }
    LOG(INFO)
        << "No ip provided, attempting to connect to the camera through USB";
    return std::string();
}

/*std::void rewritePathsInConfigJson(std:string jsonPath, std::string newPath)
{
    / Parse config.json
    std::string config = jsonPath;
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
}*/

std::shared_ptr<Camera> initCamera(int argc, char **argv) {

    Status status = Status::OK;
    std::string ip = parseArgs(argc, argv);

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    if (ip.empty()) {
        system.getCameraList(cameras);
    } else {
        system.getCameraListAtIp(cameras, ip);
    }

    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return nullptr;
    }


    std::shared_ptr<Camera> camera = cameras.front();

    // user can pass any config.json stored anywhere in HW
    status = camera->setControl("initialization_config", "config/config_walden_nxp.json");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set the initialization config file!";
        return 0;
    }

    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    status = camera->setControl("powerUp", "call");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not PowerUp camera!";
        return 0;
    }

    // optionally load configuration data from module memory
    status = camera->setControl("loadModuleData", "call");
    if (status != Status::OK) {
        LOG(INFO) << "No CCB/CFG data found in camera module,";
        LOG(INFO) << "Loading calibration(ccb) and configuration(cfg) data from JSON config file...";
    }

    //set depthCompute to on or off
    status = camera->setControl("enableDepthCompute", "on");
    if (status != Status::OK){
        LOG(ERROR) << "Couldn't set depth compute option";
        return 0;
    }


    status = camera->setFrameType("pcm");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 0;
    }
    aditof::Frame frame;

    return camera;
}
void startCamera(const std::shared_ptr<aditof::Camera> &camera) {
    Status status = Status::OK;

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return;
    }
    return;
}

void setFrameType(const std::shared_ptr<aditof::Camera> &camera,
                  const std::string &type) {
    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return;
    }

    std::vector<std::string>::iterator it =
        std::find(frameTypes.begin(), frameTypes.end(), type);
    if (it == frameTypes.end()) {
        LOG(ERROR) << "Requested frame type is not available";
        return;
    }

    Status status = Status::OK;
    status = camera->setFrameType(type);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return;
    }
}

void getAvailableFrameType(const std::shared_ptr<aditof::Camera> &camera,
                           std::vector<std::string> &availableFrameTypes) {
    camera->getAvailableFrameTypes(availableFrameTypes);
    if (availableFrameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return;
    }
}

void setMode(const std::shared_ptr<aditof::Camera> &camera,
             const std::string &mode) {
    std::vector<std::string> modes;
    camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "No camera modes available!";
        return;
    }

    std::vector<std::string>::iterator it =
        std::find(modes.begin(), modes.end(), mode);
    if (it == modes.end()) {
        LOG(ERROR) << "Requested mode is not available";
        return;
    }

    Status status = Status::OK;
    status = camera->setMode(mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return;
    }
}

void setCameraRevision(const std::shared_ptr<aditof::Camera> &camera,
                       const std::string rev) {
    Status status = Status::OK;
    status = camera->setControl("revision", rev);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera revision!";
        return;
    }
}

void setIrGammaCorrection(const std::shared_ptr<aditof::Camera> &camera,
                          float gamma) {
    Status status = Status::OK;
    status = camera->setControl("ir_gamma_correction", std::to_string(gamma));
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set ir gamma correction!";
        return;
    }
}

void applyNoiseReduction(const std::shared_ptr<Camera> &camera, int threshold) {

    Status status = Status::OK;
    status = camera->setControl("noise_reduction_threshold",
                                std::to_string(threshold));
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set noise reduction!";
        return;
    }
}

void disableNoiseReduction(const std::shared_ptr<Camera> &camera) {

    Status status = Status::OK;
    status = camera->setControl("noise_reduction_threshold", std::to_string(0));
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set noise reduction!";
        return;
    }
}

void getNewFrame(const std::shared_ptr<Camera> &camera, aditof::Frame *frame) {
    Status status = Status::OK;

    try {
        std::lock_guard<std::mutex> lck(mtx_dynamic_rec);
        status = camera->requestFrame(frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
        }
    } catch (std::exception &e) {
    }
}

uint16_t *getFrameData(aditof::Frame *frame, const std::string &dataType) {
    uint16_t *frameData;
    Status status = Status::OK;
    status = frame->getData(dataType, &frameData);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
        return nullptr;
    }

    if (!frameData) {
        LOG(ERROR) << "no memory allocated in frame";
        return nullptr;
    }
    return frameData;
}

IntrinsicParameters getIntrinsics(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.intrinsics;
}

int getRangeMax(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.maxDepth;
}

int getRangeMin(const std::shared_ptr<Camera> &camera) {
    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);
    return cameraDetails.minDepth;
}

void irTo16bitGrayscale(uint16_t *frameData, int width, int height) {
    std::vector<uint16_t> data(frameData, frameData + width * height);

    auto min_val = std::min_element(data.begin(), data.end());
    auto max_val = std::max_element(data.begin(), data.end());
    uint16_t delta = *max_val - *min_val;
    int minColorValue = 0;

    if (delta == 0) {
        return;
    }

    for (int i = 0; i < width * height; i++) {
        float norm_val = static_cast<float>(data[i] - *min_val) / delta;
        float grayscale_val =
            norm_val * std::numeric_limits<unsigned short int>::max() +
            (1.0f - norm_val) * minColorValue;
        frameData[i] = static_cast<uint16_t>(grayscale_val);
    }
}

