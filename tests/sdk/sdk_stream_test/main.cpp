/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/

#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/frame_handler.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <chrono>
#include <command_parser.h>
#include <ctime>
#include <fstream>

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#include <cstring>
#define __STDC_FORMAT_MACROS 1
#include <inttypes.h>
#endif
#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

enum : uint16_t {
    MAX_FILE_PATH_SIZE = 512,
};

using namespace aditof;

#ifdef _WIN32
int main(int argc, char *argv[]);
#endif

static const char kUsagePublic[] =
    R"(SDK Stream Test 1.1
    Usage: 
      sdk_stream_test --i <test file> --f <output older path> [--ip <ip>]
      sdk_stream_test (-h | --help)

    Options:
      -h --help          Show this screen.
      --f <folder path>  Output folder path
      --ip <ip>          Camera IP
      --i <test file>    Test definition file
)";

std::vector<std::map<std::string, std::string>>
parseCSV(const std::string &filename) {
    std::ifstream file(filename);
    std::vector<std::map<std::string, std::string>> data;
    std::vector<std::string> headers;

    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << "\n";
        return data;
    }

    std::string line;

    // Read the header
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string col;
        while (std::getline(ss, col, ',')) {
            headers.push_back(col);
        }
    }

    // Read the rest of the lines
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::map<std::string, std::string> row;
        size_t i = 0;

        while (std::getline(ss, cell, ',')) {
            if (i < headers.size()) {
                row[headers[i]] = cell;
            }
            ++i;
        }
        for (size_t idx = i; idx < static_cast<uint32_t>(headers.size());
             ++idx) {
            if (row.find(headers[idx]) == row.end()) {
                row[headers[idx]] =
                    ""; // Fill missing columns with empty string
            }
        }

        data.push_back(row);
    }

    return data;
}

int main(int argc, char *argv[]) {
    std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", "", false}},
        {"-f", {"--f", false, "", "", true}},
        {"-ip", {"--ip", false, "", "", true}},
        {"-i", {"--i", false, "", "", true}}};

    CommandParser command;
    std::string arg_error;
    command.parseArguments(argc, argv, command_map);

    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0 || argc == 1) {
        std::cerr << kUsagePublic;
        return -1;
    }

    result = command.helpMenu();
    if (result == 1) {
        std::cerr << kUsagePublic;
        return 0;
    } else if (result == -1) {
        std::cerr << "Usage of argument -h/--help"
                  << " is incorrect! Help argument should be used alone!";
        return -1;
    }

    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        std::cerr << "Argument: " << command_map[arg_error].long_option
                  << " doesn't have assigned or default value!";
        LOG(INFO) << kUsagePublic;
        return -1;
    }

    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        std::cerr << "Mandatory argument: " << arg_error << " missing";
        return -1;
    }

    result = command.checkMandatoryPosition(command_map, arg_error);
    if (result != 0) {
        std::cerr << "Mandatory argument " << arg_error
                  << " is not on its correct position ("
                  << command_map[arg_error].position << ").";
        return -1;
    }

    char folder_path[MAX_FILE_PATH_SIZE]; // Path to store the depth frames

    uint16_t err = 0;
    std::string ip;
    std::string configuration = "standard";

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;

	bool saveOutput = false;

    if (!command_map["-f"].value.empty()) {
        saveOutput = true;
        // Parsing output folder
        err = snprintf(folder_path, sizeof(folder_path), "%s",
            command_map["-f"].value.c_str());
        if (err < 0) {
            LOG(ERROR) << "Error copying the output folder path!";
            return -1;
        }
#ifdef _WIN32
        // Create folder if not created already
        char dir_path[MAX_PATH];
        if (GetFullPathName(folder_path, MAX_PATH, &dir_path[0], NULL) == 0) {
            LOG(ERROR) << "Error Unable to get directory. Error:" << GetLastError();
            return -1;
        }

        if (!(CreateDirectory(dir_path, NULL))) {
            if (ERROR_ALREADY_EXISTS != GetLastError()) {
                LOG(ERROR) << "Error creating directory. Error:", GetLastError();
                return -1;
            }
        }

#else
        err = mkdir(folder_path, 0777);

        if (err < 0) {
            LOG(ERROR) << "Unable to create directory";
            return -1;
        }
#endif
    }   

    // Parsing ip
    if (!command_map["-ip"].value.empty()) {
        ip = command_map["-ip"].value;
    }

    //Parsing test description file path
    std::string descFilePath;
    if (!command_map["-i"].value.empty()) {
        descFilePath = command_map["-i"].value;
    }
    auto testVectors = parseCSV(descFilePath);

    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    if (!ip.empty()) {
        ip = "ip:" + ip;
        system.getCameraList(cameras, ip);
    } else {
        system.getCameraList(cameras);
    }

    if (cameras.empty()) {
        LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                   << ",LN" << __LINE__ << ",DN:Camera not found";
        return -2;
    }

    auto getField = [](const char *testname, const uint32_t lineNumber,
                       const std::map<std::string, std::string> &row,
                       const std::string &fieldName) -> std::string {
        auto it = row.find(fieldName);
        if (it == row.end()) {
            LOG(ERROR) << "@@," << testname << ",FAIL"
                       << ",LN" << lineNumber << ",DN:Error with " << fieldName;
            return "";
        }
        return it->second;
    };

    auto camera = cameras.front();

	uint32_t cntr = 2; // Start from 2 to match the test vector numbering
    for (const auto& row : testVectors) {
        std::string modeStr = getField(argv[0], __LINE__, row, "mode");
        if (modeStr.empty()) {
            continue;
        }
        uint32_t mode = std::stoi(modeStr);

        std::string rtmsStr = getField(argv[0], __LINE__, row, "rtms");
        if (rtmsStr.empty()) {
            continue;
        }
        std::chrono::milliseconds capture_time_milliseconds(std::stoi(rtmsStr));

        std::string fpsStr = getField(argv[0], __LINE__, row, "fps");
        if (fpsStr.empty()) {
            continue;
        }
        uint16_t fps = std::stoi(fpsStr);

        std::string cfgStr = getField(argv[0], __LINE__, row, "cfg");


        std::string cntrStr = std::to_string(cntr);
        cntr++;

        const std::string summary =
            cntrStr + ":" + modeStr + ":" + rtmsStr + ":" + fpsStr + ":" + cfgStr;

        uint32_t n_frames = 0;

        status = camera->initialize(cfgStr);
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                << ",LN" << __LINE__
                << ",DN:Could not initialize camera:TST:" << summary;
            return -2;
        }

        status = camera->setSensorConfiguration(configuration);
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                << ",LN" << __LINE__
                << ",DN:Could not configure camera with "
                << configuration << ":TST:" << summary;
			return -2;
        }

        aditof::CameraDetails cameraDetails;
        camera->getDetails(cameraDetails);

        LOG(INFO) << "SD card image version: "
            << cameraDetails.sdCardImageVersion;
        LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
        LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

        // Get modes
        std::vector<uint8_t> availableModes;
        status = camera->getAvailableModes(availableModes);
        if (status != Status::OK || availableModes.empty()) {
            LOG(ERROR) << "Could not aquire modes";
            return -2;
        }

        status = camera->setMode(static_cast<uint8_t>(mode));
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                << ",LN" << __LINE__
                << ",DN:Could not set camera mode:TST:" << summary;
            return -2;
        }

        char time_buffer[128];
        time_t rawtime;
        time(&rawtime);
        struct tm timeinfo;
#ifdef _WIN32
        localtime_s(&timeinfo, &rawtime);
#else
        localtime_r(&rawtime, &timeinfo);
#endif
        strftime(time_buffer, sizeof(time_buffer), "%Y%m%d%H%M%S", &timeinfo);

        camera->adsd3500SetFrameRate(fps);

        // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
        status = camera->start();
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                << ",LN" << __LINE__
                << ",DN:Could not start camera:TST:" << summary;
            return -2;
        }

        aditof::Frame frame;
        FrameDetails fDetails;

        auto warmup_start = std::chrono::steady_clock::now();

        FrameHandler frameSaver;
        if (saveOutput) {
            frameSaver.storeFramesToSingleFile(true);
            frameSaver.setOutputFilePath(folder_path);
        }

        //drop first frame
        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                       << ",LN" << __LINE__
                       << ",DN:Could not request frame:TST:" << summary;
            status = camera->stop();
            if (status != Status::OK) { // Result already given, not test result error message
                LOG(ERROR) << argv[0] << ",FAIL"
                    << ",LN" << __LINE__
                    << ",DNError stopping camera:TST:" << summary;
            }
            return -2;
        }

        Metadata metadata;

        status = frame.getMetadataStruct(metadata);
        if (status != Status::OK) {
            LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                       << ",LN" << __LINE__
                       << ",DN:Could not get metadata:TST:" << summary;
            status = camera->stop();
            if (status != Status::OK) { // Result already given, not test result error message
                LOG(ERROR) << argv[0] << ",FAIL"
                    << ",LN" << __LINE__
                    << ",DNError stopping camera:TST:" << summary;
            }
            return -2;
        }

        uint32_t frameNumber = metadata.frameNumber;

        uint32_t expectNumberOfFrames =
            (static_cast<uint32_t>(capture_time_milliseconds.count()) * fps) /
            1000;

        uint32_t divisor = (expectNumberOfFrames * 20) / 100;

        uint32_t frame_count = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        if (capture_time_milliseconds.count() > 0) {
            LOG(INFO) << "Capturing frames for "
                      << capture_time_milliseconds.count()
                      << " milliseconds in mode " << mode << "!";
            auto capture_duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    capture_time_milliseconds);
            while (std::chrono::high_resolution_clock::now() - start_time <
                   capture_duration) {

                if ((frame_count % divisor) == 0) {
                    LOG(INFO) << __func__ << ": framecount: " << frame_count;
                }

                status = camera->requestFrame(&frame);
                if (status != Status::OK) {
                    LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                               << ",LN" << __LINE__
                               << ",DN:Could not request frame:TST:" << summary;
                    status = camera->stop();
                    if (status != Status::OK) { // Result already given, not test result error message
                        LOG(ERROR) << argv[0] << ",FAIL"
                            << ",LN" << __LINE__
                            << ",DNError stopping camera:TST:" << summary;
                    }
                    return -3;
                }

                status = frame.getMetadataStruct(metadata);
                if (status != Status::OK) {
                    LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                               << ",LN" << __LINE__
                               << ",DN:Could not get metadata:TST:" << summary;
                    status = camera->stop();
                    if (status != Status::OK) { // Result already given, not test result error message
                        LOG(ERROR) << argv[0] << ",FAIL"
                            << ",LN" << __LINE__
                            << ",DNError stopping camera:TST:" << summary;
                    }
                    return -3;
                }

                if (metadata.frameNumber != frameNumber + 1) {
                    LOG(ERROR) << "@@," << argv[0] << ",FAIL"
                               << ",LN" << __LINE__
                               << ",DN:Frame number mismatch expected "
                               << (frameNumber + frame_count) << " but got "
                               << metadata.frameNumber << ":" << summary;
                    status = camera->stop();
                    if (status != Status::OK) { // Result already given, not test result error message
                        LOG(ERROR) << argv[0] << ",FAIL"
                            << ",LN" << __LINE__
                            << ",DNError stopping camera:TST:" << summary;
                    }
                    return -3;
                }

                frameNumber = metadata.frameNumber;

                if (saveOutput && (frame_count % divisor) == 0) {
                    frameSaver.saveFrameToFile(frame);
                }

                frame_count++;
            }
            n_frames = frame_count;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_time = end_time - start_time;
        if (frame_count > 0) {
            double measured_fps = (double)n_frames / total_time.count();
            LOG(INFO) << "@@," << argv[0] << ",PASS"
                      << ",LN" << __LINE__ << ",DN:FPS:" << measured_fps
                      << ":FC:" << frame_count << ":TST:" << summary;
        } else {
            LOG(INFO) << "@@," << argv[0] << ",FAIL"
                      << ",LN" << __LINE__ << ",DN:FC:" << frame_count
                      << ":TST:" << summary;
            return -3;
        }

        status = camera->stop();
        if (status != Status::OK) { // Result already given, not test result error message
            LOG(ERROR) << argv[0] << ",FAIL"
                       << ",LN" << __LINE__
                       << ",DNError stopping camera:TST:" << summary;
            return -3;
        }

        for (int i = 2; i > 0; --i) {
            LOG(INFO) << "\rSleeping for " << i << " seconds...   "
                      << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return 0;
}