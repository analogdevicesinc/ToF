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

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
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

    int nFrames = 50;
    LOG(INFO) << "Requesting " << nFrames << " frames!";

    auto start_time = std::chrono::high_resolution_clock::now();

    // Request the frames for the respective mode
    for (uint32_t loopcount = 0; loopcount < nFrames; loopcount++) {

        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

    } // End of for Loop

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_time = end_time - start_time;
    if (total_time.count() > 0.0) {
        double measured_fps = (double)nFrames / total_time.count();
        LOG(INFO) << "Measured FPS: " << measured_fps;
    }



    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 1;
    }

    return 0;
}