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

int case_netLinkTest(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
                     std::string param_val) {
    Status status = Status::OK;
    
    std::string netlinktest_s = param_val;
    status = depthSensor->setControl("netlinktest", netlinktest_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set netlinktest value";
        return 1;
    }
    LOG(INFO) << "netlink test value set to " << netlinktest_s;

    std::string netlinktest_g;
    status = depthSensor->getControl("netlinktest", netlinktest_g);
    if (status == Status::OK) {
        LOG(ERROR) << "Error: netlinktest is write-only control";
        return 1;
    } 
    LOG(INFO) << "Netlinktest is write-only control";

    return 0;
}

int case_inputFormat(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
                     std::string param_val) {
    Status status = Status::OK;

    std::string inputFormat_s = param_val;
    status = depthSensor->setControl("inputFormat", inputFormat_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set inputFormat value";
        return 1;
    }
    LOG(INFO) << "inputFormat test value set to " << inputFormat_s;

    std::string inputFormat_g;
    status = depthSensor->getControl("inputFormat", inputFormat_g);
    if (status == Status::OK) {
        LOG(ERROR) << "Error: inputFormat is write-only control";
        return 1;
    } 
    LOG(INFO) << "inputFormat is write-only control";

    return 0;
}

int case_fps(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
             std::string param_val) {
    Status status = Status::OK;

    std::string fps_s = param_val;
    status = depthSensor->setControl("fps", fps_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set fps value";
        return 1;
    }
    LOG(INFO) << "fps value set to " << fps_s;

    std::string fps_g;
    status = depthSensor->getControl("fps", fps_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannot get FPS value";
        return 1;
    } 
    LOG(INFO) << "FPS is write only control";

    return 0;
}

int case_modeInfoVersion(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::string param_val) {
    Status status = Status::OK;

    std::string modeInfo_s = param_val;
    status = depthSensor->setControl("modeInfoVersion", modeInfo_s);
    if (status != Status::OK) {
        LOG(ERROR) << "ERROR! modeInfoVersion not set.";
        return 1;
    }
    LOG(INFO) << "modeInfoVersion is  set";

    std::string modeInfo_g;
    status = depthSensor->getControl("modeInfoVersion", modeInfo_g);
    if (status != Status::OK) {
        LOG(ERROR) << "cannot get modeInfoVersion value";
        return 1;
    }
    LOG(INFO) << "modeInfo version: " << modeInfo_g;

    return 0;
}

int case_confidenceBits(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::string param_val) {
    Status status = Status::OK;

    std::string confidenceBits_s = param_val;
    status = depthSensor->setControl("confidenceBits", confidenceBits_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set confidenceBits value";
        return 1;
    }
    LOG(INFO) << "ConfidenceBits value set to: " << confidenceBits_s;

    std::string confidenceBits_g;
    status = depthSensor->getControl("confidenceBits", confidenceBits_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get confidenceBits value";
        LOG(INFO) << status;
        return 1;
    }
    LOG(INFO) << "confidenceBits: " << confidenceBits_g;

    return 0;
}

int case_ImagerType(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
                    std::string param_val) {
    Status status = Status::OK;

    std::string imagerType_s = param_val;
    status = depthSensor->setControl("imagerType", imagerType_s);
    if (status == Status::OK) {
        LOG(ERROR) << "ERROR! imagerType is should not be changed.";
        return 1;
    }
    LOG(INFO) << "imagerType is  read only";

    std::string imagerType_g;
    status = depthSensor->getControl("imagerType", imagerType_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get imagerType value";
        return 1;
    }
    LOG(INFO) << "imagerType: " << imagerType_g;

    return 0;
}

int case_ABBits(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::string param_val) {
    Status status = Status::OK;

    std::string abBits_s = param_val;
    status = depthSensor->setControl("abBits", abBits_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set abBits value";
        return 1;
    }
    LOG(INFO) << "abBits value set to: " << abBits_s;

    std::string abBits_g;
    status = depthSensor->getControl("abBits", abBits_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get abBits value";
        LOG(INFO) << status;
        return 1;
    }
    LOG(INFO) << "abBits: " << abBits_g;

    return 0;
}

int case_phaseDepthBits(
    std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
    std::string param_val) {
    Status status = Status::OK;

    std::string phaseDepthBits_s = param_val;
    status = depthSensor->setControl("phaseDepthBits", phaseDepthBits_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set phaseDepthBits value";
        return 1;
    }
    LOG(INFO) << "phaseDepthBits value set to: " << phaseDepthBits_s;

    std::string phaseDepthBits_g;
    status = depthSensor->getControl("phaseDepthBits", phaseDepthBits_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get phaseDepthBits value";
        LOG(INFO) << status;
        return 1;
    }
    LOG(INFO) << "phaseDepthBits: " << phaseDepthBits_g;

    return 0;
}

int case_depthEnable(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
                     std::string param_val) {
    Status status = Status::OK;

    std::string depthEnable_s = param_val;
    status = depthSensor->setControl("depthEnable", depthEnable_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set depthEnable value";
        return 1;
    }
    LOG(INFO) << "depthEnable value set to: " << depthEnable_s;

    std::string depthEnable_g;
    status = depthSensor->getControl("depthEnable", depthEnable_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get depthEnable value";
        LOG(INFO) << status;
        return 1;
    }
    LOG(INFO) << "depthEnable: " << depthEnable_g;

    return 0;
}

int case_abAveraging(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
                     std::string param_val) {
    Status status = Status::OK;
    
    std::string abAveraging_s = param_val;
    status = depthSensor->setControl("abAveraging", abAveraging_s);
    if (status != Status::OK) {
        LOG(INFO) << "cannot set abAveraging value";
        return 1;
    }
    LOG(INFO) << "abAveraging value set to: " << abAveraging_s;

    std::string abAveraging_g;
    status = depthSensor->getControl("abAveraging", abAveraging_g);
    if (status != Status::OK) {
        LOG(INFO) << "cannot get abAveraging value";
        LOG(INFO) << status;
        return 1;
    }
    LOG(INFO) << "abAveraging: " << abAveraging_g;

    return 0;
}

int main(int argc, char *argv[]) {

    enum myCases {
        NETLINKTEST,
        INPUTFORMAT,
        FPS,
        MODEINFOVERSION,
        CONFIDENCEBITS,
        IMAGERTYPE,
        ABBITS,
        PHASEDEPTHBITS,
        DEPTHENABLE,
        ABAVERAGING
    };

    int test_case;
    std::string ip;
    std::string param_val;
    if (argc > 3) {
        ip = argv[1];
        test_case = std::stoi(argv[2]);
        param_val = argv[3];
    } else {
        ip = "10.42.0.1";
        test_case = ABAVERAGING;
        param_val = "0";
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

    std::shared_ptr<aditof::DepthSensorInterface> depthSensor = camera->getSensor();

    status = depthSensor->open();
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return 1;
    }

    LOG(INFO) << "Available Control: ";
    std::vector<std::string> controls;
    status = depthSensor->getAvailableControls(controls);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to retrive frame types!";
        return 1;
    }
    for (const std::string &str : controls) {
        std::cout << str << ", ";
    }
    std::cout << std::endl;

    int return_val;

    switch (test_case) {
    case MODEINFOVERSION:
        return_val = case_modeInfoVersion(depthSensor, param_val);
        break;

    case CONFIDENCEBITS:
        return_val = case_confidenceBits(depthSensor, param_val);
        break;

    case IMAGERTYPE:
        return_val = case_ImagerType(depthSensor, param_val);
        break;

    case ABBITS:
        return_val = case_ABBits(depthSensor, param_val);
        break;

    case PHASEDEPTHBITS:
        return_val = case_phaseDepthBits(depthSensor, param_val);
        break;

    case DEPTHENABLE:
        return_val = case_depthEnable(depthSensor, param_val);
        break;

    case ABAVERAGING:
        return_val = case_abAveraging(depthSensor, param_val);
        break;

    case NETLINKTEST:
        return_val = case_netLinkTest(depthSensor, param_val);
        break;

    case INPUTFORMAT:
        return_val = case_inputFormat(depthSensor, param_val);
        break;

    case FPS:
        return_val = case_fps(depthSensor, param_val);
        break;

    default:
        return_val = 1;
        break;
    }


    return return_val;
}