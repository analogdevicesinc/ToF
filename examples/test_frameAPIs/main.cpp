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
#include <chrono>
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
    if (argc > 3) {
        // convert the string argument to an integer
        modeNum = std::stoi(argv[1]);
        ip = argv[2];
        configFile = argv[3];

    } else {
        // set num to default: 0(sr-native)
        modeNum = 0;
        ip = "10.42.0.1";
        configFile = "config/config_adsd3500_adsd3100.json";
    }

    LOG(INFO) << "value: " << modeNum;

    Status status = Status::OK;
    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

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
    
    std::string modeName;
    camera->getFrameTypeNameFromId(modeNum, modeName);
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
    }

    aditof::FrameDetails frameDetails;
    frame.getDetails(frameDetails);

    LOG(INFO) << "fDetailsFrameType: " << frameDetails.type;
    LOG(INFO) << "fDetailsCameraMode: " << frameDetails.cameraMode;
    LOG(INFO) << "fDetailsWidth: " << frameDetails.width;
    LOG(INFO) << "fDetailsHeight: " << frameDetails.height;
    LOG(INFO) << "fDetailsTotalCapture: "
              << static_cast<int>(frameDetails.totalCaptures);
    LOG(INFO) << "fDetailsPassiveIRCaptured: " << frameDetails.passiveIRCaptured;

    aditof::FrameDataDetails frameDataDetails;
    std::string types[] = {
        "ab", "depth", "conf", "xyz", "metadata"};
    for (const std::string &type : types) {
        frame.getDataDetails(type, frameDataDetails);

        LOG(INFO) << type << "-type: " << frameDataDetails.type;
        LOG(INFO) << type << "-width: " << frameDataDetails.width;
        LOG(INFO) << type << "-height: " << frameDataDetails.height;
        LOG(INFO) << type
                  << "-subelementSize: " << frameDataDetails.subelementSize;
        LOG(INFO) << type << "-subelementsPerElement: "
                  << frameDataDetails.subelementsPerElement;
        LOG(INFO) << type << "-bytesCount: " << frameDataDetails.bytesCount;
    }

    LOG(INFO) << "Available attributes: ";
    std::vector<std::string> attributes;
    frame.getAvailableAttributes(attributes);
    if (attributes.empty()) {
        LOG(INFO) << "no attributes avaialble!";
        return 1;
    } else {
        std::cout << "Attributes: ";
        for (const auto &str : attributes) {
            std::cout << str << ' ';
        }
        std::cout << std::endl;
    }

    std::string hdr_length;
    frame.getAttribute("embed_hdr_length", hdr_length);
    LOG(INFO) << "embed_hdr_length: " << hdr_length;

    std::string embed_height;
    frame.getAttribute("embed_height", embed_height);
    LOG(INFO) << "embed_height: " << embed_height;

    std::string embed_width;
    frame.getAttribute("embed_width", embed_width);
    LOG(INFO) << "embed_width: " << embed_width;

    std::string height;
    frame.getAttribute("height", height);
    LOG(INFO) << "attrib_height: " << height;

    std::string mode;
    frame.getAttribute("mode", mode);
    LOG(INFO) << "attrib_mode: " << mode;

    std::string passive_ir;
    frame.getAttribute("passive_ir", passive_ir);
    LOG(INFO) << "passive_ir: " << passive_ir;

    std::string subframes;
    frame.getAttribute("subframes", subframes);
    LOG(INFO) << "subframes: " << subframes;

    std::string total_captures;
    frame.getAttribute("total_captures", total_captures);
    LOG(INFO) << "total_captures: " << total_captures;

    std::string width;
    frame.getAttribute("width", width);
    LOG(INFO) << "attrib_width: " << width;

    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 1;
    }

    /*
    LOG(INFO) << "========================================================";
    std::vector<std::string> controls;
    camera->getAvailableControls(controls);
    if (controls.empty()) {
        LOG(INFO) << "no control avaialble!";
        return 1;
    }
    // Print the contents using a range-based for-loop
    for (const auto &control : controls) {
        LOG(INFO) << control << 'y';
    }
    LOG(INFO) << "========================================================";
    */

    return 0;
}