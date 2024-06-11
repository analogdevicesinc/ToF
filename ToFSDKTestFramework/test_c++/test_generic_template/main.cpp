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

//using general template to access ABinvalidation threshold
int case_ABinvalidation(std::shared_ptr<Camera> camera) {
    Status status = Status::OK;
    int abThreshMinValue;
    uint16_t abthreshMinValue_g;
    int test_abThreshMinValue = 5;

    LOG(INFO) << "Getting AB invalidation threshold...";

    status = camera->adsd3500GetABinvalidationThreshold(abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive abThreshMin value!";
        return 1;
    }
    LOG(INFO) << "abThreshMin via adsd3500GetABinvalidationThreshold: "
              << abThreshMinValue;

    status = camera->adsd3500GetGenericTemplate(0x0015, abthreshMinValue_g);
    if (status != Status::OK) {
        LOG(ERROR)
            << "Cannnot retreive abThreshMin value via generic template!";
        return 1;
    }
    LOG(INFO) << "abThreshMin via generic template: " << abthreshMinValue_g;

    //set AB invaldation threshold via generic template
    status = camera->adsd3500SetGenericTemplate(0x0010, test_abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set abThreshMin value via generic template!";
        return 1;
    }
    LOG(INFO) << "abThreshMin set";

    LOG(INFO) << "validating AB invalidation threshold after setting";

    status = camera->adsd3500GetABinvalidationThreshold(abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive abThreshMin value!";
        return 1;
    }
    LOG(INFO)
        << "abThreshMin via adsd3500GetABinvalidationThreshold after setting: "
        << abThreshMinValue;

    status = camera->adsd3500GetGenericTemplate(0x0015, abthreshMinValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive abThreshMin value via generic template "
                      "after setting:";
        return 1;
    }
    LOG(INFO) << "abThreshMin via generic template after setting: " << abthreshMinValue_g;

    return 0;
}

//using general template to access frame rate
int case_framerate(std::shared_ptr<Camera> camera) {
    Status status = Status::OK;
    uint16_t frameRateValue;
    uint16_t frameRateValue_g;
    int test_frameRateValue = 30;

    LOG(INFO) << "Getting frameRate...";

    status = camera->adsd3500GetFrameRate(frameRateValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive frameRate value!";
        return 1;
    }
    LOG(INFO) << "frameRate via adsd3500GetFrameRate: "
              << frameRateValue;

    status = camera->adsd3500GetGenericTemplate(0x0023, frameRateValue_g);
    if (status != Status::OK) {
        LOG(ERROR)
            << "Cannnot retreive frameRate value via generic template!";
        return 1;
    }
    LOG(INFO) << "frameRate via generic template: " << frameRateValue_g;

    //set frameRate via generic template
    status = camera->adsd3500SetGenericTemplate(0x0022, test_frameRateValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set frameRate value via generic template!";
        return 1;
    }
    LOG(INFO) << "frameRate set";

    LOG(INFO) << "validating frameRate after setting";

    status = camera->adsd3500GetFrameRate(frameRateValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive frameRate value!";
        return 1;
    }
    LOG(INFO)
        << "frameRate via adsd3500GetFrameRate after setting: "
        << frameRateValue;

    status = camera->adsd3500GetGenericTemplate(0x0023, frameRateValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive frameRate value via generic template "
                      "after setting:";
        return 1;
    }
    LOG(INFO) << "frameRate via generic template: after setting: " << frameRateValue_g;

    return 0;
}

//using general template to access EnableMetadatainAB
int case_EnableMetadatainAB(std::shared_ptr<Camera> camera) {
    Status status = Status::OK;
    uint16_t EnableMetadatainABValue;
    uint16_t EnableMetadatainABValue_g;
    int test_EnableMetadatainABValue = 0;

    LOG(INFO) << "Getting EnableMetadatainAB...";

    status = camera->adsd3500GetEnableMetadatainAB(EnableMetadatainABValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via adsd3500GetEnableMetadatainAB: "
              << EnableMetadatainABValue;

    status =
        camera->adsd3500GetGenericTemplate(0x0037, EnableMetadatainABValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value via generic template!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via generic template: "
              << EnableMetadatainABValue_g;

    //set EnableMetadatainAB via generic template
    status = camera->adsd3500SetGenericTemplate(0x0036,
                                                test_EnableMetadatainABValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set EnableMetadatainAB value via generic template!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB set";

    LOG(INFO) << "validating EnableMetadatainAB after setting";

    status = camera->adsd3500GetEnableMetadatainAB(EnableMetadatainABValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB value  via adsd3500GetEnableMetadatainAB after setting: "
              << EnableMetadatainABValue;

    status =
        camera->adsd3500GetGenericTemplate(0x0037, EnableMetadatainABValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value via generic template "
                      "after setting:";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via generic template after setting: "
              << EnableMetadatainABValue_g;

    return 0;
}

//using get general template to access a read register(EnableMetadatainAB)
int case_use_get_on_write_register(std::shared_ptr<Camera> camera) {
    Status status = Status::OK;
    uint16_t EnableMetadatainABValue_g;
    int test_EnableMetadatainABValue = 0;

    LOG(INFO) << "Setting EnableMetadatainAB to 0...";

    //set EnableMetadatainAB via generic template
    status = camera->adsd3500SetGenericTemplate(0x0036,
                                                test_EnableMetadatainABValue);
    if (status != Status::OK) {
        LOG(ERROR)
            << "Cannnot set EnableMetadatainAB value via generic template!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB set";

    status =
        camera->adsd3500GetGenericTemplate(0x0036, EnableMetadatainABValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value via generic "
                      "template!";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via generic template: "
              << EnableMetadatainABValue_g;

    return 0;
}

//using get general template to access a read register(EnableMetadatainAB)
int case_use_set_on_read_register(std::shared_ptr<Camera> camera) {
    Status status = Status::OK;
    uint16_t EnableMetadatainABValue_g;
    int test_EnableMetadatainABValue = 0;

    LOG(INFO) << "Getting EnableMetadatainAB...";

    //get EnableMetadatainAB via generic template
    status =
        camera->adsd3500GetGenericTemplate(0x0037, EnableMetadatainABValue_g);
    if (status != Status::OK) {
        LOG(ERROR)
            << "Cannnot retreive EnableMetadatainAB value via generic template ";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via generic template: "
              << EnableMetadatainABValue_g;

    status = camera->adsd3500SetGenericTemplate(0x0037,
                                                test_EnableMetadatainABValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set EnableMetadatainAB value via generic template on output register";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB is set to 0 via generic template output register";

    status =
        camera->adsd3500GetGenericTemplate(0x0037, EnableMetadatainABValue_g);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive EnableMetadatainAB value via generic "
                      "template ";
        return 1;
    }
    LOG(INFO) << "EnableMetadatainAB via generic template after setting: "
              << EnableMetadatainABValue_g;

    return 0;
}


int main(int argc, char *argv[]) {

    enum myCases {
        ABINVALIDATION,
        FRAMERATE,
        ENABLEMETADATAINAB,
        USE_GET_ON_WRITE_REGISTER,
        USE_SET_ON_READ_REGISTER
    };

    std::string configFile;
    int modeNum;
    std::string ip;
    int test_case;
    // check argument for modeName
    if (argc > 4) {
        // convert the string argument to an integer
        modeNum = std::stoi(argv[1]);
        ip = argv[2];
        configFile = argv[3];
        test_case = std::stoi(argv[4]);
    } else {
        // set num to default: 0(sr-native)
        modeNum = 0;
        ip = "10.43.0.1";
        configFile = "config/config_adsd3500_adsd3100.json";
        test_case = ABINVALIDATION;
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

    #ifdef VERSION_5_1_0_HIGHER
    std::vector<uint8_t> availableModes;
    camera->getAvailableModes(availableModes);
    if (availableModes.empty()) {
        std::cout << "no mode available!";
        return 1;
    }
    status = camera->setMode(modeNum);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 1;
    }
    #elif VERSION_5_1_0_LOWER

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
    LOG(INFO) << "mode name: " << modeName;
    #endif

    int return_val;
    switch (test_case) {
    case ABINVALIDATION:
        return_val = case_ABinvalidation(camera);
        break;

    case FRAMERATE:
        return_val = case_framerate(camera);
        break;

    case ENABLEMETADATAINAB:
        return_val = case_EnableMetadatainAB(camera);
        break;

    case USE_GET_ON_WRITE_REGISTER:
        return_val = case_use_get_on_write_register(camera);
        break;

    case USE_SET_ON_READ_REGISTER:
        return_val = case_use_set_on_read_register(camera);
        break;

    default:
        return_val = 1;
        break;
    }
    
    return return_val;
}
