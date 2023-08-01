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

using namespace aditof;

static const char Help_Menu[] =
    R"(First-frame usage:
    first-frame CONFIG
    first-frame (-h | --help)
    first-frame [-ip | --ip <ip>] [-m | --m <mode>] CONFIG

    Arguments:
      CONFIG            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help          Show this screen.
      -m --m <mode>      Mode to capture data in. [default: 0]

    NOTE: -m | --m argument supports both index and string (0/sr-native)

    Valid mode (-m | --m) options are:
        0: short-range native
        1: long-range native
        2: short-range Qnative
        3: long-range Qnative
        4: pcm-native
        5: long-range mixed
        6: short-range mixed
)";

Status save_frame(aditof::Frame &frame, std::string frameType) {

    uint16_t *data1;
    FrameDataDetails fDetails;
    Status status = Status::OK;

    status = frame.getData(frameType, &data1);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data " + frameType + "!";
        return status;
    }

    if (!data1) {
        LOG(ERROR) << "no memory allocated in frame";
        return status;
    }

    std::ofstream g("out_" + frameType + "_" + fDetails.type + ".bin",
                    std::ios::binary);
    frame.getDataDetails(frameType, fDetails);
    g.write((char *)data1, fDetails.width * fDetails.height * sizeof(uint16_t));
    g.close();

    return status;
}

int main(int argc, char *argv[]) {
    std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", ""}},
        {"-ip", {"--ip", false, "", ""}},
        {"-m", {"--m", false, "", "0"}},
        {"config", {"CONFIG", true, "last", ""}}};

    CommandParser command;
    std::string arg_error;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    command.parseArguments(argc, argv, command_map);
    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0) {
        std::cout << "Argument " << arg_error << " doesn't exist."
                  << "\n"
                  << Help_Menu;
        return -1;
    }
    result = command.helpMenu();
    if (result == 1) {
        std::cout << Help_Menu;
        return 0;
    } else if (result == -1) {
        std::cout << "Usage of argument -h/--help"
                  << " is incorrect! Help argument should be used alone."
                  << "\n"
                  << Help_Menu;
        return -1;
    }
    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        std::cout << "Argument: " << arg_error
                  << " doesn't have assigned or default value."
                  << "\n"
                  << Help_Menu;
        return -1;
    }
    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        std::cout << "Mandatory argument: " << arg_error << " missing. \n"
                  << Help_Menu;
        return -1;
    }
    result = command.checkMandatoryPosition(command_map, arg_error);
    if (result != 0) {
        std::cout << "Mandatory argument " << arg_error
                  << " is not on its correct position ("
                  << command_map[arg_error].position << "). \n"
                  << Help_Menu;
        return -1;
    }

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;
    std::string configFile;
    std::string ip;
    uint32_t mode = 0;

    // Parsing mode type
    std::string modeName;
    try {
        std::size_t counter;
        mode = std::stoi(command_map["-m"].value, &counter);
        if (counter != command_map["-m"].value.size()) {
            throw command_map["-m"].value.c_str();
        }
    } catch (const char *name) {
        modeName = name;
    } catch (const std::exception &) {
        modeName = command_map["-m"].value;
    }

    configFile = command_map["config"].value;

    if (!command_map["-ip"].value.empty()) {
        ip = "ip:" + command_map["-ip"].value;
    }

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    if (!ip.empty()) {
        system.getCameraList(cameras, ip);
    } else {
        system.getCameraList(cameras);
    }
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();

    status = camera->setControl("initialization_config", configFile);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set control!";
        return 0;
    }

    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type avaialble!";
        return 0;
    }

    if (modeName.empty()) {
        status = camera->getFrameTypeNameFromId(mode, modeName);
        if (status != Status::OK) {
            LOG(ERROR) << "Mode: " << mode
                       << " is invalid for this type of camera!";
            return 0;
        }
    }

    status = camera->setFrameType(modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 0;
    }
    aditof::Frame frame;

    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    } else {
        LOG(INFO) << "succesfully requested frame!";
    }

    save_frame(frame, "ir");
    save_frame(frame, "depth");

    // Example of reading temperature from hardware
    uint16_t sensorTmp = 0;
    uint16_t laserTmp = 0;
    status = camera->adsd3500GetSensorTemperature(sensorTmp);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not read sensor temperature!";
    }

    status = camera->adsd3500GetLaserTemperature(laserTmp);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not read laser temperature!";
    }

    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 0;
    }

    LOG(INFO) << "Sensor temperature: " << sensorTmp;
    LOG(INFO) << "Laser temperature: " << laserTmp;

    return 0;
}
