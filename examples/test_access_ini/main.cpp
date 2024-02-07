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

    int modeNum;
    // check argument for modeName
    if (argc > 1) {
        // convert the string argument to an integer
        modeNum = std::stoi(argv[1]);
    } else {
        // set num to default: 0(sr-native)
        modeNum = 0;
    }
    
    LOG(INFO) << "value: " << modeNum;

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

    std::string modeName;
    camera->getFrameTypeNameFromId(modeNum, modeName);
    status = camera->setFrameType(modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 1;
    }
    LOG(INFO) << "mode name: " << modeName;

    int abThreshMinValue;
    int test_abThreshMinValue = 5;
    status = camera->adsd3500GetABinvalidationThreshold(abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive abThreshMin value!";
        return 1;
    }
    LOG(INFO) << "abThreshMin: " << abThreshMinValue;

    status = camera->adsd3500SetABinvalidationThreshold(test_abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set abThreshMin value!";
        return 1;
    }
    LOG(INFO) << "abThreshMin value set ";

    status = camera->adsd3500GetABinvalidationThreshold(abThreshMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot get abThreshMin value!";
        return 1;
    }
    if (abThreshMinValue != test_abThreshMinValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }
    LOG(INFO) << "abThreshMin value after: " << abThreshMinValue;

    int confThreshValue;
    int test_confThreshValue = 20;
    status = camera->adsd3500GetConfidenceThreshold(confThreshValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive confThresh value!";
        return 1;
    }
    LOG(INFO) << "confThresh: " << confThreshValue;

    status = camera->adsd3500SetConfidenceThreshold(test_confThreshValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set confThresh value!";
        return 1;
    }
    LOG(INFO) << "confThresh value set ";

    status = camera->adsd3500GetConfidenceThreshold(confThreshValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set confThresh value!";
        return 1;
    }
    LOG(INFO) << "confThresh value after: " << confThreshValue;

    if (confThreshValue != test_confThreshValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    int RadialThresholdMinValue;
    int test_RadialThresholdMinValue = 20;
    status = camera->adsd3500GetRadialThresholdMin(RadialThresholdMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive RadialThresholdMin value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMin: " << RadialThresholdMinValue;

    status =
        camera->adsd3500SetRadialThresholdMin(test_RadialThresholdMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set RadialThresholdMin value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMin value set ";

    status = camera->adsd3500GetRadialThresholdMin(RadialThresholdMinValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set RadialThresholdMin value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMin value after: " << RadialThresholdMinValue;

    if (RadialThresholdMinValue != test_RadialThresholdMinValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    int RadialThresholdMaxValue;
    int test_RadialThresholdMaxValue = 17000;
    status = camera->adsd3500GetRadialThresholdMax(RadialThresholdMaxValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive RadialThresholdMax value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMax: " << RadialThresholdMaxValue;

    status =
        camera->adsd3500SetRadialThresholdMax(test_RadialThresholdMaxValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set RadialThresholdMax value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMax value set ";

    status = camera->adsd3500GetRadialThresholdMax(RadialThresholdMaxValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set RadialThresholdMax value!";
        return 1;
    }
    LOG(INFO) << "RadialThresholdMax value after: " << RadialThresholdMaxValue;
    
    if (RadialThresholdMaxValue != test_RadialThresholdMaxValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    bool jblfApplyFlagValue;
    bool test_jblfApplyFlagValue = 0;
    status = camera->adsd3500GetJBLFfilterEnableState(jblfApplyFlagValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive jblfApplyFlag value!";
        return 1;
    }
    LOG(INFO) << "jblfApplyFlag: " << jblfApplyFlagValue;

    status = camera->adsd3500SetJBLFfilterEnableState(test_jblfApplyFlagValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfApplyFlag value!";
        return 1;
    }
    LOG(INFO) << "jblfApplyFlag value set ";

    status = camera->adsd3500GetJBLFfilterEnableState(jblfApplyFlagValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfApplyFlag value!";
        return 1;
    }
    LOG(INFO) << "jblfApplyFlag value after: " << jblfApplyFlagValue;

    if (jblfApplyFlagValue != test_jblfApplyFlagValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }
    
    int jblfWindowSizeValue;
    int test_jblfWindowSizeValue = 5;
    status = camera->adsd3500GetJBLFfilterSize(jblfWindowSizeValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive jblfWindowSize value!";
        return 1;
    }
    LOG(INFO) << "jblfWindowSize: " << jblfWindowSizeValue;

    status = camera->adsd3500SetJBLFfilterSize(test_jblfWindowSizeValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfWindowSize value!";
        return 1;
    }
    LOG(INFO) << "jblfWindowSize value set ";

    status = camera->adsd3500GetJBLFfilterSize(jblfWindowSizeValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfWindowSize value!";
        return 1;
    }
    LOG(INFO) << "jblfWindowSize value after: " << jblfWindowSizeValue;

    if (jblfWindowSizeValue != test_jblfWindowSizeValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    uint16_t JBLFGaussianSigmaValue;
    uint16_t test_JBLFGaussianSigmaValue = 12;
    status = camera->adsd3500GetJBLFGaussianSigma(JBLFGaussianSigmaValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive JBLFGaussianSigma value!";
        return 1;
    }
    LOG(INFO) << "JBLFGaussianSigma: " << JBLFGaussianSigmaValue;

    status = camera->adsd3500SetJBLFGaussianSigma(test_JBLFGaussianSigmaValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set JBLFGaussianSigma value!";
        return 1;
    }
    LOG(INFO) << "JBLFGaussianSigma value set ";

    status = camera->adsd3500GetJBLFGaussianSigma(JBLFGaussianSigmaValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set JBLFGaussianSigma value!";
        return 1;
    }
    LOG(INFO) << "JBLFGaussianSigma value after: " << JBLFGaussianSigmaValue;

    if (JBLFGaussianSigmaValue != test_JBLFGaussianSigmaValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    uint16_t jblfExponentialTermValue;
    uint16_t test_jblfExponentialTermValue = 12;
    status = camera->adsd3500GetJBLFExponentialTerm(jblfExponentialTermValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive jblfExponentialTerm value!";
        return 1;
    }
    LOG(INFO) << "jblfExponentialTerm: "
              << jblfExponentialTermValue;

    status =
        camera->adsd3500SetJBLFExponentialTerm(test_jblfExponentialTermValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfExponentialTerm value!";
        return 1;
    }
    LOG(INFO) << "jblfExponentialTerm value set ";

    status = camera->adsd3500GetJBLFExponentialTerm(jblfExponentialTermValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfExponentialTerm value!";
        return 1;
    }
    LOG(INFO) << "jblfExponentialTerm value after: "
              << jblfExponentialTermValue;

    if (jblfExponentialTermValue != test_jblfExponentialTermValue) {
        LOG(ERROR) << "value set is not value get!";
        return 1;
    }

    uint16_t jblfMaxEdgeValue;
    status = camera->adsd3500SetJBLFMaxEdgeThreshold(12);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfMaxEdge value!";
        return 1;
    }
    LOG(INFO) << "jblfMaxEdge value set ";


    
    uint16_t test_jblfABThresholdValue = 12;
    status = camera->adsd3500SetJBLFABThreshold(test_jblfABThresholdValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set jblfABThreshold value!";
        return 1;
    }
    LOG(INFO) << "jblfABThreshold value set ";

    //header size
    //deltaCompEnable
    //inputFormat
    //depthComputeIspEnable
    //partialDepthEnable
    //xyzEnanle

    uint16_t fpsValue;
    status = camera->adsd3500GetFrameRate(fpsValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot retreive fps value!";
        return 1;
    }
    LOG(INFO) << "fps: " << fpsValue;

    status = camera->adsd3500SetFrameRate(12);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set fps value!";
        return 1;
    }
    LOG(INFO) << "fps value set ";

    status = camera->adsd3500GetFrameRate(fpsValue);
    if (status != Status::OK) {
        LOG(ERROR) << "Cannnot set fps value!";
        return 1;
    }
    LOG(INFO) << "fps value after: " << fpsValue;

    return 0;
}