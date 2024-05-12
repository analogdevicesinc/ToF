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

    std::string sensorName;
    status = depthSensor->getName(sensorName);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to open device";
        return 1;
    }
    LOG(INFO) << "Sensor Name: " << sensorName;

    aditof::SensorDetails sensorDetails;
    status = depthSensor->getDetails(sensorDetails);
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to get sensor details";
        return 1;
    }
    LOG(INFO) << "Sensor ID: " << sensorDetails.id;
    std::string connectionType[] = {"ON_TARGET", "USB", "NETWORK", "OFFLINE"};
    if (sensorDetails.connectionType == ConnectionType::ON_TARGET) {
        LOG(INFO) << "Connection Type: ON_TARGET";
    } else if (sensorDetails.connectionType == ConnectionType::USB) {
        LOG(INFO) << "Connection Type: USB";
    } else if (sensorDetails.connectionType == ConnectionType::NETWORK) {
        LOG(INFO) << "Connection Type: NETWORK";
    } else if (sensorDetails.connectionType == ConnectionType::OFFLINE) {
        LOG(INFO) << "Connection Type: OFFLINE";
    }

    std::string imagerType;
    std::string controlValue;
    status = depthSensor->getControl("imagerType", controlValue);

    if (status == Status::OK) {
        if (controlValue == "1") {
            imagerType = "ADSD3100";
        } else if (controlValue == "2") {
            imagerType = "ADSD3030";
        } else {
            imagerType = "UNSET";
            LOG(INFO) << "Unkown imager type: ";
        }
        LOG(INFO) << "Imager type: " << imagerType;
    }
    
    std::vector<aditof::DepthSensorFrameType> availableSensorFrameTypes;
    status = depthSensor->getAvailableFrameTypes(availableSensorFrameTypes);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to retrive frame types!";
        return 1;
    }

    for (size_t index = 0; index < availableSensorFrameTypes.size(); ++index) {
        LOG(INFO) << "++++++++++++++++++++++++++++++";
        LOG(INFO) << "frame type: " << availableSensorFrameTypes[index].type;
        LOG(INFO) << "frame width: " << availableSensorFrameTypes[index].height;
        LOG(INFO) << "frame height: " << availableSensorFrameTypes[index].width;

        for (size_t index1 = 0; index1 < availableSensorFrameTypes[index].content.size();
             ++index1) {
            LOG(INFO) << "frame content type: " << availableSensorFrameTypes[index].content[index1].type;
            LOG(INFO) << availableSensorFrameTypes[index].content[index1].type <<
                   " width: " << availableSensorFrameTypes[index].content[index1].height;
            LOG(INFO) << availableSensorFrameTypes[index].content[index1].type <<
                " height: " << availableSensorFrameTypes[index].content[index1].width;
        }
    }

    return 0;
}