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
    R"(Data Collect.
    Usage:
      data_collect 
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--wt <warmup>] [--ccb FILE] [--ip <ip>] [--fw <firmware>] [-s | --split] [-t | --netlinktest] [--ic <imager-configuration>] [-scf <save-configuration-file>] [-lcf <load-configuration-file>]
      data_collect (-h | --help)

    Options:
      -h --help          Show this screen.
      --f <folder>       Output folder (max name 512) [default: ./]
      --n <ncapture>     Capture frame num. [default: 1]
      --m <mode>         Mode to capture data in. [default: 0]
      --wt <warmup>      Warmup Time (sec) [default: 0]
      --ccb <FILE>       The path to store CCB content
      --ip <ip>          Camera IP
      --fw <firmware>    Adsd3500 fw file
      --split            Save each frame into a separate file (Debug)
      --netlinktest      Puts server on target in test mode (Debug)
      --singlethread     Store the frame to file using same tread
      --ic <imager-configuration>   Select imager configuration: standard, standard-raw,
                         custom, custom-raw. By default is standard.
      --scf <save-configuration-file>    Save current configuration to json file
      --lcf <load-configuration-file>    Load configuration from json file

    Note: --m argument supports index (0, 1, etc.) 

    Valid mode (--m) options are:
        0: short-range native
        1: long-range native
        2: short-range Qnative
        3: long-range Qnative
        4: pcm-native
        5: long-range mixed
        6: short-range mixed
)";

int main(int argc, char *argv[]) {
    std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", "", false}},
        {"-f", {"--f", false, "", ".", true}},
        {"-n", {"--n", false, "", "0", true}},
        {"-m", {"--m", false, "", "0", true}},
        {"-wt", {"--wt", false, "", "0", true}},
        {"-ip", {"--ip", false, "", "", true}},
        {"-fw", {"--fw", false, "", "", true}},
        {"-ccb", {"--ccb", false, "", "", true}},
        {"-s", {"--split", false, "", "", false}},
        {"-t", {"--netlinktest", false, "", "", false}},
        {"-st", {"--singlethread", false, "", "", false}},
        {"-ic", {"--ic", false, "", "", true}},
        {"-scf", {"--scf", false, "", "", true}},
        {"-lcf", {"--lcf", false, "", "", true}}};

    CommandParser command;
    std::string arg_error;

    command.parseArguments(argc, argv, command_map);

    if (argc == 1) {
        std::cout << kUsagePublic << std::endl;
        return 0;
    }

    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument " << arg_error << " doesn't exist! "
                   << "Please check help menu.";
        return -1;
    }

    result = command.helpMenu();
    if (result == 1) {
        std::cout << kUsagePublic << std::endl;
        return 0;
    } else if (result == -1) {
        LOG(ERROR) << "Usage of argument -h/--help"
                   << " is incorrect! Help argument should be used alone!";
        return -1;
    }

    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument: " << command_map[arg_error].long_option
                   << " doesn't have assigned or default value!";
        LOG(INFO) << kUsagePublic;
        return -1;
    }

    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Mandatory argument: " << arg_error << " missing";
        LOG(INFO) << kUsagePublic;
        return -1;
    }

    result = command.checkMandatoryPosition(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Mandatory argument " << arg_error
                   << " is not on its correct position ("
                   << command_map[arg_error].position << ").";
        LOG(INFO) << kUsagePublic;
        return -1;
    }

    char folder_path[MAX_FILE_PATH_SIZE]; // Path to store the depth frames
    char json_file_path
        [MAX_FILE_PATH_SIZE]; // Get the .json file from command line

    uint16_t err = 0;
    uint32_t n_frames = 0;
    uint8_t mode = 0;
    uint32_t warmup_time = 0;
    std::string ip;
    std::string firmware;
    std::string configuration = "standard";

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;
    // Parsing the arguments from command line
    err = snprintf(json_file_path, sizeof(json_file_path), "%s",
                   command_map["-lcf"].value.c_str());
    if (err < 0) {
        LOG(ERROR) << "Error copying the json file path!";
        return 0;
    }

    // Parsing output folder
    err = snprintf(folder_path, sizeof(folder_path), "%s",
                   command_map["-f"].value.c_str());

    if (err < 0) {
        LOG(ERROR) << "Error copying the output folder path!";
        return 0;
    }
#ifdef _WIN32
    // Create folder if not created already
    char dir_path[MAX_PATH];
    if (GetFullPathName(folder_path, MAX_PATH, &dir_path[0], NULL) == 0) {
        LOG(ERROR) << "Error Unable to get directory. Error:" << GetLastError();
        return 0;
    }

    if (!(CreateDirectory(dir_path, NULL))) {
        if (ERROR_ALREADY_EXISTS != GetLastError()) {
            LOG(ERROR) << "Error creating directory. Error:", GetLastError();
            return 0;
        }
    }

#else
    err = mkdir(folder_path, 0777);

    if (err < 0) {
        LOG(ERROR) << "Unable to create directory";
        return 0;
    }
#endif

    // Parsing number of frames
    n_frames = std::stoi(command_map["-n"].value);

    if (!command_map["-m"].value.empty()) {
        mode = std::stoi(command_map["-m"].value);
    }

    // Parsing ip
    if (!command_map["-ip"].value.empty()) {
        ip = command_map["-ip"].value;
    }

    // Parsing firmware
    if (!command_map["-fw"].value.empty()) {
        firmware = command_map["-fw"].value;
    }

    //Parsing Warm up time
    if (!command_map["-wt"].value.empty()) {
        warmup_time = std::stoi(command_map["-wt"].value);
        if (warmup_time < 0) {
            LOG(ERROR) << "Invalid warm up time input!";
        }
    }

    //Parsing CCB path
    std::string ccbFilePath;
    if (!command_map["-ccb"].value.empty()) {
        ccbFilePath = command_map["-ccb"].value;
    }

    //Parsing split option
    bool saveToSingleFile = true;
    if (!command_map["-s"].value.empty()) {
        saveToSingleFile = false;
    }

    //Parsing single thread option
    bool samethread = 0;
    if (!command_map["-st"].value.empty()) {
        samethread = true;
    }

    //Parsing netLinkTest option
    bool useNetLinkTest = !command_map["-t"].value.empty();

    // Parsing configuration option
    std::vector<std::string> configurationlist = {"standard", "standard-raw",
                                                  "coustom", "custom-raw"};

    std::string configurationValue = command_map["-ic"].value;
    if (!configurationValue.empty()) {
        unsigned int pos =
            std::find(configurationlist.begin(), configurationlist.end(),
                      configurationValue) -
            configurationlist.begin();
        if (pos < configurationlist.size()) {
            configuration = configurationValue;
        }
    }

    bool saveconfigurationFile = false;
    std::string saveconfigurationFileValue = command_map["-scf"].value;
    if (!saveconfigurationFileValue.empty()) {
        if (saveconfigurationFileValue.find(".json") == std::string::npos) {
            saveconfigurationFileValue += ".json";
        }
        saveconfigurationFile = true;
        strcpy(json_file_path, "");
    }

    LOG(INFO) << "Output folder: " << folder_path;
    LOG(INFO) << "Mode: " << command_map["-m"].value;
    LOG(INFO) << "Number of frames: " << n_frames;
    LOG(INFO) << "Json file: " << json_file_path;
    LOG(INFO) << "Warm Up Time is: " << warmup_time << " seconds";
    LOG(INFO) << "Configuration is: " << configuration;

    if (!ip.empty()) {
        LOG(INFO) << "Ip address is: " << ip;
    }

    if (!firmware.empty()) {
        LOG(INFO) << "Firmware file is is: " << firmware;
    }

    if (!ccbFilePath.empty()) {
        LOG(INFO) << "Path to store CCB content: " << ccbFilePath;
    }

    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    if (!ip.empty()) {
        ip = "ip:" + ip;
        if (useNetLinkTest) {
            ip += ":netlinktest";
        }
        system.getCameraList(cameras, ip);
    } else {
        system.getCameraList(cameras);
    }

    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();

    status = camera->initialize(json_file_path);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    status = camera->setSensorConfiguration(configuration);
    if (status != Status::OK) {
        LOG(INFO) << "Could not configure camera with " << configuration;
    } else {
        LOG(INFO) << "Configure camera with " << configuration;
    }

    if (saveconfigurationFile) {
        status = camera->saveDepthParamsToJsonFile(saveconfigurationFileValue);
        if (status != Status::OK) {
            LOG(INFO) << "Could not save current configuration info to "
                      << saveconfigurationFileValue;
        } else {
            LOG(INFO) << "Current configuration info saved to file "
                      << saveconfigurationFileValue;
        }
    }

    aditof::CameraDetails cameraDetails;
    camera->getDetails(cameraDetails);

    LOG(INFO) << "SD card image version: " << cameraDetails.sdCardImageVersion;
    LOG(INFO) << "Kernel version: " << cameraDetails.kernelVersion;
    LOG(INFO) << "U-Boot version: " << cameraDetails.uBootVersion;

    if (!firmware.empty()) {
        std::ifstream file(firmware);
        if (!(file.good() &&
              file.peek() != std::ifstream::traits_type::eof())) {
            LOG(ERROR) << firmware << " not found or is an empty file";
            return 0;
        }

        status = camera->adsd3500UpdateFirmware(firmware);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not update the adsd3500 firmware";
            return 0;
        } else {
            LOG(INFO) << "Please reboot the board!";
            return 0;
        }
    }

    // Get modes
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    if (status != Status::OK || availableModes.empty()) {
        LOG(ERROR) << "Could not aquire modes";
        return 0;
    }

    std::shared_ptr<DepthSensorInterface> depthSensor = camera->getSensor();
    std::string sensorName;
    status = depthSensor->getName(sensorName);

    status = camera->setMode(mode);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return 0;
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
#if 0
    camera->setControl("setFPS", std::to_string(setfps));
    if (status != Status::OK) {
        LOG(ERROR) << "Error setting camera FPS to " << setfps;
        return 0;
    }
#endif

    // Store CCB to file
    if (!ccbFilePath.empty()) {
        status = camera->saveModuleCCB(ccbFilePath);
        if (status != Status::OK) {
            LOG(INFO) << "Failed to store CCB to " << ccbFilePath;
        }
    }

    if (n_frames == 0) {
        LOG(INFO) << n_frames << " frames requested, exiting." << std::endl;
        return -1;
    }

    // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return 0;
    }

    aditof::Frame frame;
    FrameDetails fDetails;
    uint64_t elapsed_time;

    auto warmup_start = std::chrono::steady_clock::now();

    // Wait until the warmup time is finished
    if (warmup_time > 0) {
        do {
            status = camera->requestFrame(&frame);
            if (status != Status::OK) {
                LOG(ERROR) << "Could not request frame!";
                return 0;
            }

            auto warmup_end = std::chrono::steady_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                               warmup_end - warmup_start)
                               .count();
        } while (warmup_time >= elapsed_time);
    }

    FrameHandler frameSaver;
    frameSaver.storeFramesToSingleFile(saveToSingleFile);
    frameSaver.setOutputFilePath(folder_path);

    //drop first frame
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    }

    LOG(INFO) << "Requesting " << n_frames << " frames!";
    auto start_time = std::chrono::high_resolution_clock::now();
    // Request the frames for the respective mode
    for (uint32_t loopcount = 0; loopcount < n_frames; loopcount++) {

        status = camera->requestFrame(&frame);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }
        if (useNetLinkTest) {
            continue;
        }

        if (!samethread) {
            frameSaver.saveFrameToFileMultithread(frame);
        } else {
            frameSaver.saveFrameToFile(frame);
        }
    } // End of for Loop

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_time = end_time - start_time;
    if (total_time.count() > 0.0) {
        double measured_fps = (double)n_frames / total_time.count();
        LOG(INFO) << "Measured FPS: " << measured_fps;
    }

    status = camera->stop();
    if (status != Status::OK) {
        LOG(INFO) << "Error stopping camera!";
    }
    return 0;
}
