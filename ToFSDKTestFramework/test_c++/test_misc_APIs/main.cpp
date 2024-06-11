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
#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <command_parser.h>
#include <fstream>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <ios>
#include <iostream>
#include <map>
#include <windows.h>

using namespace aditof;

int main(int argc, char *argv[]) {
    std::string configFile;
    int modeNum;
    std::string ip;
    // check argument for modeName
    if (argc > 2) {
        // convert the string argument to an integer
        ip = argv[1];
        configFile = argv[2];

    } else {
        ip = "10.43.0.1";
        configFile = "config/config_adsd3500_adsd3100.json";
    }

    Status status = Status::OK;

    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    if (!ip.empty()) {
        ip = "ip:" + ip;
        status = system.getCameraList(cameras, ip);
        std::cout << status;
    } else {
        status = system.getCameraList(cameras);
        std::cout << status;
    }
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 1;
    }

    auto camera = cameras.front();

    status = camera->initialize(configFile);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 1;
    }

    std::string fwVersion;
    std::string fwHash;
    status = camera->adsd3500GetFirmwareVersion(fwVersion, fwHash);
    if (status != Status::OK) {
        LOG(ERROR) << "failed to retrieve Firmware version!";
        return 1;
    } else {
        LOG(INFO) << "firmware version: " << fwVersion;
        LOG(INFO) << "firmware hash: " << fwHash;

    }

    std::string serialNum;
    for (int cacheValue = 0; cacheValue < 2; ++cacheValue) {
        status = camera->readSerialNumber(serialNum, cacheValue % 2 == 0);
        if (status != Status::OK) {
            LOG(ERROR) << "failed to retrieve serial number!";
            return 1;
        } else {
            LOG(INFO) << "Serial Number: " << serialNum;
        }
    }

    int chipStatus, imagerStatus;
    status = camera->adsd3500GetStatus(chipStatus, imagerStatus);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get imager error codes!";
        return 0;
    }

    LOG(INFO) << "Chip status error code: " << chipStatus;
    LOG(INFO) << "Imager status error code: " << imagerStatus;
    
    aditof::CameraDetails cameraDetails;
    status = camera->getDetails(cameraDetails);
    if (status != Status::OK) {
        LOG(ERROR) << "failed to get camera details!";
        return 1;
    }

    LOG(INFO) << "Camera ID: " << cameraDetails.cameraId;
    LOG(INFO) << "Mode: " << cameraDetails.mode;

    LOG(INFO) << "Frame Details: ";
    LOG(INFO) << "type: " << cameraDetails.frameType.type;
    LOG(INFO) << "Camera mode: " << cameraDetails.frameType.cameraMode;
    LOG(INFO) << "width: " << cameraDetails.frameType.width;
    LOG(INFO) << "height: " << cameraDetails.frameType.height;
    LOG(INFO) << "totalCaptures: " << cameraDetails.frameType.totalCaptures;
    LOG(INFO) << "passiveIRCaptured: " << cameraDetails.frameType.passiveIRCaptured;

    LOG(INFO) << "Camera Intrinsics: ";
    LOG(INFO) << "codx: " << cameraDetails.intrinsics.codx;
    LOG(INFO) << "cody: " << cameraDetails.intrinsics.cody;
    LOG(INFO) << "cx: " << cameraDetails.intrinsics.cx;
    LOG(INFO) << "cy: " << cameraDetails.intrinsics.cy;
    LOG(INFO) << "fx: " << cameraDetails.intrinsics.fx;
    LOG(INFO) << "fy: " << cameraDetails.intrinsics.fy;
    LOG(INFO) << "k1: " << cameraDetails.intrinsics.k1;
    LOG(INFO) << "k2: " << cameraDetails.intrinsics.k2;
    LOG(INFO) << "k3: " << cameraDetails.intrinsics.k3;
    LOG(INFO) << "k4: " << cameraDetails.intrinsics.k4;
    LOG(INFO) << "k5: " << cameraDetails.intrinsics.k5;
    LOG(INFO) << "k6: " << cameraDetails.intrinsics.k6;
    LOG(INFO) << "p1: " << cameraDetails.intrinsics.p1;
    LOG(INFO) << "p2: " << cameraDetails.intrinsics.p2;

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;
    LOG(INFO) << "Serial Number via Camera Details: " << cameraDetails.serialNumber;
    
    
    
    /*
    std::vector<std::string> controls;
    status = camera->getAvailableControls(controls);
    if (status != Status::OK) {
        LOG(ERROR) << "failed to get Available control!";
        return 1;
    }
    if (controls.empty()) {
        LOG(INFO) << "No control avaialble!";
        return 1;
    }
    for (const std::string &str : controls) {
        LOG(INFO) << str;
    }
    */
    return 0;
}