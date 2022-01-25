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

using namespace aditof;

bool isValidIPv4(const char *IPAddress)
{
   unsigned char a,b,c,d;
   return sscanf(IPAddress,"%d.%d.%d.%d", &a, &b, &c, &d) == 4;
}

std::string parseArgsForIp(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
    
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            std::string ip = argv[i];
            if (isValidIPv4(ip.c_str()))
                return ip;
        }
    }
    LOG(INFO)
        << "No ip provided, attempting to connect to the camera through USB";
    return std::string();
}

bool parseArgsForDepthLibarary(int argc, char **argv) {

    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            std::string var = argv[i];
            if (std::strcmp(var.c_str(),"true")==0)
                return true;
        }
    }
    LOG(INFO) << "No depth_compute option provided, default value: FALSE";
    return false;
}

std::shared_ptr<Camera> initCamera(int argc, char **argv) {

    Status status = Status::OK;
    std::string ip = parseArgsForIp(argc, argv);
    bool useDepthLibrary = parseArgsForDepthLibarary(argc, argv);

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

    /*std::string package_path = ros::package::getPath("aditof_roscpp");
    package_path = package_path + "/../../config/config_walden_nxp.json";
    LOG(INFO) << "Json config file location: "<< package_path;*/

    // user can pass any config.json stored anywhere in HW

    status = camera->setControl(
        "initialization_config",
        "/home/analog/.ros/config/config_walden_nxp.json");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set the initialization config file!";
        return 0;
    }
        
    if (!useDepthLibrary){
        status = camera->setControl("enableDepthCompute", "off");
        if (status != Status::OK) {
            LOG(ERROR) << "Could not set the initialization config file!";
            return 0;
        }
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
        LOG(INFO) << "Loading calibration(ccb) and configuration(cfg) data "
                     "from JSON config file...";
    }

    status = camera->setControl("enableDepthCompute", "off");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set depthCompute to off!";
        return 0;
    }

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
    status = camera->requestFrame(frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
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