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
#include <aditof/frame.h>
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
    Status status = Status::OK;
    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    System system;
    std::string ip = "ip:10.42.0.1";
    std::string configFile = "config/config_adsd3500_adsd3100.json";
    std::vector<std::shared_ptr<Camera>> cameras;

    if (!ip.empty()) {
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

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
        return 1;
    }

    // Check if a directory named "data_dir" exists and create it if not
    DWORD attr = GetFileAttributes("data_dir");
    if (attr == INVALID_FILE_ATTRIBUTES) {
        if (CreateDirectory("data_dir", NULL)) {
            LOG(INFO) << "Directory created successfully\n";
        } else {
            LOG(INFO) << "Error creating directory\n";
        }
    } else if (attr & FILE_ATTRIBUTE_DIRECTORY) {
        LOG(INFO) << "The directory already exists\n";
    } else {
        LOG(INFO) << "The path is not a directory\n";
    }

    std::string modeName;
    //for (int id_num = 0; id_num <= 6; id_num++)
    //{
    //    camera->getFrameTypeNameFromId(id_num, modeName);
    camera->getFrameTypeNameFromId(0, modeName);
    status = camera->setFrameType(modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 1;
    }

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 1;
    }

    aditof::Frame frame;

    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 1;
    } else {
        LOG(INFO) << "succesfully requested frame!";
    }

    std::string pixelCount;
    uint64_t frame_size = 0;
    uint16_t *pData;

    std::string frameType[] = {"ab", "depth", "conf", "metadata"};
    for (const auto &type : frameType) {
        FrameDataDetails FrameDataDetails;
        status = frame.getDataDetails(type, FrameDataDetails);
        if (status != Status::OK) {
            LOG(ERROR) << type << "disabled from ini file!";
            return 1;
        }

        uint32_t height = FrameDataDetails.height;
        uint32_t width = FrameDataDetails.width;
        uint32_t bytesCount = FrameDataDetails.bytesCount;

        if (type == "ab") {
            frame_size = sizeof(uint16_t) * height * width;
        } else if (type == "depth") {
            frame_size = sizeof(uint16_t) * height * width;
        } else if (type == "conf") {
            frame_size = sizeof(float) * height * width;
        } else if (type == "metadata") {
            frame_size = bytesCount;
        } else {
            LOG(WARNING) << "Can't recognize frame data type!";
        }

        LOG(INFO) << "Frame type: : " << type;
        LOG(INFO) << "Height: " << height;
        LOG(INFO) << "width: " << width;
        LOG(INFO) << "bytecount: " << bytesCount;

        status = frame.getData(type, &pData);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not get frame data " + type + "!";
            return 1;
        }

        if (!pData) {
            LOG(ERROR) << "no memory allocated in frame";
            return 1;
        }

        std::ofstream g("data_dir/out_" + type + "_" + modeName + ".bin",
                        std::ios::binary);

        g.write((char *)pData, frame_size);
        g.close();

        if (modeName == "pcm-native") {
            LOG(WARNING) << "pcm-native only contains ab frame";
            break;
        }
    }

    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 1;
    }
    //}

    return 0;
}