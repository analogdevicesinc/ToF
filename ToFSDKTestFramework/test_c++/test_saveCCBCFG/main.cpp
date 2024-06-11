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


//using get general template to access a read register(EnableMetadatainAB)
int case_use_saveModuleCCB(std::shared_ptr<Camera> camera, std::string ccbFileName) {
    Status status = Status::OK;

    ccbFileName.append(".ccb");
    status = camera->saveModuleCCB(ccbFileName);
    if (status != Status::OK) {
        LOG(ERROR) << "Fail to save CCB!";
        return 1;
    }
    return 0;
}

//using get general template to access a read register(EnableMetadatainAB)
int case_use_saveModuleCFG(std::shared_ptr<Camera> camera,  std::string cfgFileName) { 
    Status status = Status::OK;

    cfgFileName.append(".cfg");
    status = camera->saveModuleCFG(cfgFileName);
    if (status != Status::OK) {
        LOG(ERROR) << "Fail to save CFG!";
        return 1;
    }
    return 0; 
}

int main(int argc, char *argv[]) {

    enum myCases {
        GETCCB,
        GETCFG
    };

    std::string configFile;
    std::string ip;
    std::string fileName;
    int test_case;
    // check argument for modeName
    if (argc > 4) {
        // convert the string argument to an integer
        ip = argv[1];
        configFile = argv[2];
        fileName = argv[3];
        test_case = std::stoi(argv[4]);

    } else {
        ip = "10.43.0.1";
        configFile = "config/config_adsd3500_adsd3100.json";
        fileName = "trial";
        test_case = 0;
    }

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
    
        int return_val;
    switch (test_case) {
        case GETCCB:
            return_val = case_use_saveModuleCCB(camera, fileName);
            break;

        case GETCFG:
            return_val = case_use_saveModuleCFG(camera, fileName);
            break;


        default:
            return_val = 1;
            break;
    }

    return return_val;
}
