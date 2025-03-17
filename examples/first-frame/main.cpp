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
#include <chrono>
#include <ctime>
#include <iomanip>  // For std::setw, std::setfill

using namespace aditof;

static const char Help_Menu[] =
    R"(First-frame usage:
    first-frame
    first-frame (-h | --help)
    first-frame [-ip | --ip <ip>] [-m | --m <mode>] [-config | --config <config_file.json>]

    Arguments:
      config_file.json   Input config_default.json file (which has *.ccb and *.cfg)

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

void print_wall_clock_time(int64_t epoch_time) {
    // Convert epoch time to local time
    std::time_t time = static_cast<std::time_t>(epoch_time / 1000);  // Convert to seconds
    std::tm *tm_info = std::localtime(&time);

    if (!tm_info) {
        std::cerr << "Error: localtime() returned nullptr. Invalid epoch value?" << std::endl;
        return;
    }

    char buffer[30];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_info);

    // Extract milliseconds
    int milliseconds = epoch_time % 1000;

    std::cout << "Given 64-bit Epoch Time: " << epoch_time << std::endl;
    std::cout << "Converted Wall Clock Time: " << buffer << "." 
              << std::setw(3) << std::setfill('0') << milliseconds << std::endl;

    // Get current wall clock time from system with milliseconds
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

    std::time_t current_time = std::chrono::system_clock::to_time_t(now);
    std::tm *current_tm = std::localtime(&current_time);

    char current_buffer[30];
    std::strftime(current_buffer, sizeof(current_buffer), "%Y-%m-%d %H:%M:%S", current_tm);

    int current_milliseconds = now_ms.count() % 1000;

    std::cout << "Current Wall Clock Time (System Time): " << current_buffer << "."
              << std::setw(3) << std::setfill('0') << current_milliseconds << std::endl;
}

int main(int argc, char *argv[]) {
    std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", "", false}},
        {"-ip", {"--ip", false, "", "", true}},
        {"-m", {"--m", false, "", "0", true}},
        {"-config", {"--config", false, "last", "", false}}};

    CommandParser command;
    std::string arg_error;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    // parse arguments
    command.parseArguments(argc, argv, command_map);
    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument " << arg_error << " doesn't exist!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.helpMenu();
    if (result == 1) {
        std::cout << Help_Menu;
        return 0;
    } else if (result == -1) {
        LOG(ERROR) << "Usage of argument -h/--help"
                   << " is incorrect! Help argument should be used alone!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument: " << command_map[arg_error].long_option
                   << " doesn't have assigned or default value!";
        std::cout << Help_Menu;
        return -1;
    }

    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Mandatory argument missing";
        std::cout << Help_Menu;
        return -1;
    }

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;
    std::string configFile;
    std::string ip;
    uint8_t mode = 0;

    if (!command_map["-m"].value.empty()) {
        mode = std::stoi(command_map["-m"].value);
    }

    if (!command_map["-config"].value.empty()) {
        configFile = command_map["-config"].value;
    }

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

    // Registering a callback to be executed when ADSD3500 issues an interrupt
    std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
    aditof::SensorInterruptCallback callback = [](Adsd3500Status status) {
        LOG(INFO) << "Running the callback for which the status of ADSD3500 "
                     "has been "
                     "forwarded. ADSD3500 status = "
                  << status;
    };
    Status registerCbStatus =
        sensor->adsd3500_register_interrupt_callback(callback);
    if (status != Status::OK) {
        LOG(WARNING) << "Could not register callback";
    }

    if (!configFile.empty()) {
        status = camera->initialize(configFile);
    } else {
        status = camera->initialize();
    }

    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

    std::vector<uint8_t> availableModes;
    camera->getAvailableModes(availableModes);
    if (availableModes.empty()) {
        std::cout << "no mode available!";
        return 0;
    }

    status = camera->setMode(mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    status = camera->adsd3500SetFrameRate(10);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
    }

    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 0;
    }
    aditof::Frame frame;

    for (int32_t idx = 0; idx < 5; idx++) {
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        } else {
            LOG(INFO) << "succesfully requested frame!";
        }

        Metadata metadata;
        status = frame.getMetadataStruct(metadata);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not get frame metadata!";
            return 0;
        }

        uint64_t *epoch_time = (uint64_t *)&metadata.elapsedTimeFractionalValue;
        print_wall_clock_time(*epoch_time);
        LOG(INFO) << "Sensor Temperature: " << metadata.sensorTemperature;
        LOG(INFO) << "Laser Temperature: " << metadata.laserTemperature;
        LOG(INFO) << "Frame Number: " << metadata.frameNumber;
        LOG(INFO) << "Mode: " << static_cast<unsigned int>(metadata.imagerMode);

        //save_frame(frame, "ab");
        //save_frame(frame, "depth");
    }

    status = camera->stop();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not stop the camera!";
        return 0;
    }

    // Example of reading status of ADSD3500 chip and of imager
    int chipStatus, imagerStatus;
    status = camera->adsd3500GetStatus(chipStatus, imagerStatus);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not get imager error codes!";
        return 0;
    }

    LOG(INFO) << "Chip status error code: " << chipStatus;
    LOG(INFO) << "Imager status error code: " << imagerStatus;

    // Example on how to unregister a callback from ADSD3500 interupts
    if (registerCbStatus == Status::OK) {
        sensor->adsd3500_unregister_interrupt_callback(callback);
    }

    return 0;
}
