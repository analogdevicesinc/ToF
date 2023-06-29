/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/

#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
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
    FRAME_TYPE_LEN = 20,
};

#define MULTI_THREADED 1
#define DATA_COLLECT_VERSION "1.3.0"
#define EMBED_HDR_LENGTH 128

using namespace aditof;

#ifdef _WIN32
int main(int argc, char *argv[]);
#endif

typedef struct thread_params {
    uint16_t *pCaptureData;
    uint8_t *pHeaderData;
    uint16_t nframes;
    uint32_t nFrameNum;
    uint64_t nTotalCaptureSize;
    uint64_t nFrameCount;
    uint64_t pFramesize;
    const char *pFolderPath;
    const char *pFrame_type;
    const char *nFileTime;
} thread_params;

static const char kUsagePublic[] =
    R"(Data Collect.
    Usage:
      data_collect CONFIG
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--ext_fsync <0|1>] [--wt <warmup>] [--ccb FILE] [--ip <ip>] [--fw <firmware>] CONFIG
      data_collect (-h | --help)

    Arguments:
      CONFIG            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help          Show this screen.
      --f <folder>       Output folder (max name 512) [default: ./]
      --n <ncapture>     Capture frame num. [default: 1]
      --m <mode>         Mode to capture data in. [default: 0]
      --ext_fsync <0|1>  External FSYNC [0: Internal 1: External] [default: 0]
      --wt <warmup>      Warmup Time (sec) [default: 0]
      --ccb <FILE>       The path to store CCB content
      --ip <ip>          Camera IP
      --fw <firmware>    Adsd3500 fw file

    Valid mode (--m) options are:
        0: short-range native
        1: long-range native
        2: short-range Qnative
        3: long-range Qnative
        4: pcm-native
        5: long-range mixed
        6: short-range mixed
)";

#ifdef MULTI_THREADED
void fileWriterTask(const thread_params *const pThreadParams);
#endif

int main(int argc, char *argv[]) {
    std::map<std::vector<std::string>, std::string> command_map = {
        {{"-h", "--help"}, ""},         {{"-f", "--f"}, "."},
        {{"-n", "--n"}, "1"},           {{"-m", "--m"}, "0"},
        {{"-ext", "--ext_fsync"}, "0"}, {{"-wt", "--wt"}, ""},
        {{"-ip", "--ip"}, ""},          {{"-fw", "--fw"}, ""},
        {{"-fps", "--fps"}, ""},        {{"-ccb", "--ccb"}, ""},
        {{"-ft", "--ft"}, "raw"},       {{"CONFIG", "config"}, ""}};
    CommandParser command;
    command.parseArguments(argc, argv);
    std::vector<std::pair<std::string, std::string>> arg_vector =
        command.getConfiguration();
    bool is_command;

    // Goes through vector of pairs and assings values in map
    for (int it = 0; it < arg_vector.size(); it++) {
        if (arg_vector[it].second == "help_menu") {
            if (it != 0 || arg_vector.size() != 1) {
                LOG(ERROR) << "Usage of argument " + arg_vector[it].first +
                                  " is incorrect! " + arg_vector[it].first
                           << " should be used alone!";
                return 0;
            }
            LOG(INFO) << kUsagePublic;
            return 0;
        }
        is_command = false;
        if (it == arg_vector.size() - 1) {
            if (arg_vector[it].first != "CONFIG" &&
                arg_vector[it].first != "config") {
                LOG(ERROR) << "Argument CONFIG is missing! "
                           << "Please check help menu.";
                return 0;
            }
        }
        for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
            if (arg_vector[it].first == ct->first[0] ||
                arg_vector[it].first == ct->first[1]) {
                ct->second = arg_vector[it].second;
                is_command = true;
                break;
            }
        }
        if (!is_command) {
            if (arg_vector[it].second.find('-') == 0) {
                LOG(ERROR) << "Argument CONFIG should be the last one! "
                           << "Please check help menu.";
                return 0;
            }
            LOG(ERROR) << "Argument " + arg_vector[it].first +
                              " does not exist! "
                       << "Please check help menu.";
            return 0;
        }
    }

    char folder_path[MAX_FILE_PATH_SIZE]; // Path to store the raw/depth frames
    char json_file_path
        [MAX_FILE_PATH_SIZE]; // Get the .json file from command line
    std::string frame_type; // Type of frame need to be captured (Raw/Depth/IR)

    uint16_t err = 0;
    uint32_t n_frames = 0;
    uint32_t mode = 0;
    uint32_t ext_frame_sync_en = 0;
    uint32_t warmup_time = 0;
    std::string ip;
    std::string firmware;
    uint32_t setfps = 0;
    uint16_t fps_defaults[11] = {200, 105, 100, 200, 50, 50,
                                 50,  105, 105, 50,  50};

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "SDK version: " << aditof::getApiVersion()
              << " | branch: " << aditof::getBranchVersion()
              << " | commit: " << aditof::getCommitVersion();

    Status status = Status::OK;

    // Parsing the arguments from command line
    err = snprintf(json_file_path, sizeof(json_file_path), "%s",
                   command_map[{"CONFIG", "config"}].c_str());
    if (err < 0) {
        LOG(ERROR) << "Error copying the json file path!";
        return 0;
    }

    // Parsing output folder
    err = snprintf(folder_path, sizeof(folder_path), "%s",
                   command_map[{"-f", "--f"}].c_str());

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
    n_frames = std::stoi(command_map[{"-n", "--n"}]);

    // Parsing mode type
    mode = std::stoi(command_map[{"-m", "--m"}]);

    // Parsing ip
    if (!command_map[{"-ip", "--ip"}].empty()) {
        ip = command_map[{"-ip", "--ip"}];
    }

    // Parsing firmware
    if (!command_map[{"-fw", "--fw"}].empty()) {
        firmware = command_map[{"-fw", "--fw"}];
    }

    //set FPS value
    if (!command_map[{"-fps", "--fps"}].empty()) {
        setfps = std::stoi(command_map[{"-fps", "--fps"}]);
    } else {
        setfps = fps_defaults[mode];
    }

    ext_frame_sync_en = std::stoi(command_map[{"-ext", "--ext_fsync"}]);

    frame_type = command_map[{"-ft", "--ft"}];

    if (frame_type.length() <= 0) {
        LOG(ERROR) << "Error parsing frame_type (-ft/--ft) from command line!"
                   << "\n Please check help menu";
        return 0;
    }

    //Parsing Warm up time
    if (!command_map[{"-wt", "--wt"}].empty()) {
        warmup_time = std::stoi(command_map[{"-wt", "--wt"}]);
        if (warmup_time < 0) {
            LOG(ERROR) << "Invalid warm up time input!";
        }
    }

    //Parsing CCB path
    std::string ccbFilePath;
    if (!command_map[{"-ccb", "--ccb"}].empty()) {
        ccbFilePath = command_map[{"-ccb", "--ccb"}];
    }

    LOG(INFO) << "Output folder: " << folder_path;
    LOG(INFO) << "Mode: " << mode;
    LOG(INFO) << "Number of frames: " << n_frames;
    LOG(INFO) << "Json file: " << json_file_path;
    LOG(INFO) << "Frame type is: " << frame_type;
    LOG(INFO) << "Warm Up Time is: " << warmup_time << " seconds";

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
    }
    system.getCameraList(cameras, ip);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();

    // user can pass any config.json stored anywhere in HW
    status = camera->setControl("initialization_config", json_file_path);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set the initialization config file!";
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

    if (!firmware.empty()) {

        std::ifstream file(firmware);
        if (!(file.good() &&
              file.peek() != std::ifstream::traits_type::eof())) {
            LOG(ERROR) << firmware << " not found or is an empty file";
            return 0;
        }

        status = camera->setControl("updateAdsd3500Firmware", firmware);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not update the adsd3500 firmware";
            return 0;
        } else {
            LOG(INFO) << "Please reboot the board!";
            return 0;
        }
    }

    // Get frame types
    std::vector<std::string> frameTypes;
    status = camera->getAvailableFrameTypes(frameTypes);
    if (status != Status::OK || frameTypes.empty()) {
        LOG(ERROR) << "Could not aquire frame types";
        return 0;
    }

    std::string modeName;
    status = camera->getFrameTypeNameFromId(mode, modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Mode: " << mode
                   << " is invalid for this type of camera!";
        return 0;
    }

    std::shared_ptr<DepthSensorInterface> depthSensor = camera->getSensor();
    std::string sensorName;
    status = depthSensor->getName(sensorName);

    // Set UVC format type and camera frame details
    if ("raw" == frame_type) {
        camera->setControl("enableDepthCompute", "off");
    } else if ("depth" == frame_type) {
        if (modeName == "pcm-native") {
            LOG(ERROR) << modeName
                       << " mode doesn't contain depth data, please set --ft "
                          "(frameType) to raw.";
            return 0;
        } else {
            camera->setControl("enableDepthCompute", "on");
        }
    } else {
        LOG(ERROR) << "unsupported frame type!";
        return 0;
    }

    status = camera->setFrameType(modeName);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
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
        status = camera->setControl("saveModuleCCB", ccbFilePath);
        if (status != Status::OK) {
            LOG(INFO) << "Failed to store CCB to " << ccbFilePath;
        }
    }

    // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera
    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return 0;
    }

    if (ext_frame_sync_en == 0) {
        status = camera->setControl("syncMode", "0, 0"); // Master, timer driven

    } else if (ext_frame_sync_en == 1) {
        status = camera->setControl(
            "syncMode",
            "2, 0"); // Slave, 1.8v  // TODO: This configuration is required by oFilm, expand for finer control
    }

    aditof::Frame frame;
    FrameDetails fDetails;
    std::string frameType;

    uint16_t *frameBuffer;
    uint8_t *headerBuffer;
    uint32_t height;
    uint32_t width;
    uint32_t subFrames = 0;
    uint64_t frame_size = 0;
    uint64_t elapsed_time;

    auto warmup_start = std::chrono::steady_clock::now();

    // Wait until the warmup time is finished
    if (warmup_time > 0) {
        do {
            frameType = "raw";
            uint16_t *pRawFrame;
            status = camera->requestFrame(&frame);
            if (status != Status::OK) {
                LOG(ERROR) << "Could not request frame!";
                return 0;
            }
            status = frame.getData(frameType, &pRawFrame);
            if (status != Status::OK) {
                LOG(ERROR) << "Could not get Raw frame type data!";
                return 0;
            }

            auto warmup_end = std::chrono::steady_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                               warmup_end - warmup_start)
                               .count();
        } while (warmup_time >= elapsed_time);
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

        frame.getDetails(fDetails);

        height = fDetails.height;
        width = fDetails.width;

        std::string imagerType;
        status = camera->getControl("imagerType", imagerType);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to get imager type!";
            return 0;
        }

        // We have both 8bit and 16bit pixels, compute the size in 8bit
        if (sensorName == "adsd3500") {
            if (imagerType == "1") {
                if (modeName == "lr-native" || modeName == "mp") {
                    subFrames = 8;
                } else if (modeName == "sr-native") {
                    subFrames = 6;
                } else {
                    subFrames = 5;
                }
            } else {
                subFrames = 5;
            }
        } else {
            std::string attrVal;
            frame.getAttribute("total_captures", attrVal);
            subFrames = std::stoi(attrVal);
            //here it was 16 bit. changed to 8 bit
            subFrames = subFrames * 2;
        }

        if (modeName == "pcm-native") {
            subFrames = 2;
        }

        frameType = "raw";

        // Depth Data
        if (frame_type == "depth") {
            frame_size = sizeof(uint16_t) * height * width;
            frameType = "depth";
        } else if (frame_type == "raw") {
            frame_size = height * width * subFrames;
            frameType = "raw";
        } else {
            LOG(WARNING) << "Can't recognize frame data type!";
        }

        /* Since getData returns the pointer to the depth/raw data, which only main thread has access to,
            we need to copy depth frame to local memory and pass that to thread for file I/O, there is no drop in the throughput with this. */
        frameBuffer = new uint16_t[frame_size];
        if (frameBuffer == NULL) {
            LOG(ERROR) << "Can't allocate Memory for frame type data!";
            return 0;
        }

        uint16_t *pData;
        status = frame.getData(frameType, &pData);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not get frame type data!";
            return 0;
        }
        if (!pData) {
            LOG(ERROR) << "no memory allocated in frame";
            return 0;
        }
        memcpy(frameBuffer, (uint8_t *)pData, frame_size);

        uint32_t header_data_size = EMBED_HDR_LENGTH * subFrames;
        headerBuffer = new uint8_t[header_data_size];
        if (headerBuffer == NULL) {
            LOG(ERROR) << "Can't allocate Memory for frame header data!";
            return 0;
        }

#if 0 // TO DO: uncomment this one the header becomes available
        uint16_t *pHeader = nullptr;
        status = frame.getData("embeded_header", &pHeader);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not get frame header data!";
            return 0;
        }
        if (!pHeader) {
            LOG(ERROR) << "no memory allocated in frame header";
            return 0;
        }
        memcpy(headerBuffer, (uint8_t *)pHeader, header_data_size);
#endif

        // Create thread to handle the file I/O of copying raw/depth images to file
#ifdef MULTI_THREADED
        thread_params *pThreadParams = new thread_params();
        if (pThreadParams == nullptr) {
            LOG(ERROR) << "Thread param memory allocation failed";
            return 0;
        }
        pThreadParams->pCaptureData = frameBuffer;
        pThreadParams->pHeaderData = headerBuffer;
        pThreadParams->nTotalCaptureSize = frame_size;
        pThreadParams->pFolderPath = folder_path;
        pThreadParams->nFileTime = time_buffer;
        pThreadParams->nframes = n_frames;
        pThreadParams->nFrameCount = loopcount;
        pThreadParams->pFrame_type = frame_type.c_str();
        pThreadParams->pFramesize = height * width;

        /* fileWriterThread handles the copying of raw/depth frames to a file */
        std::thread fileWriterThread(
            fileWriterTask,
            const_cast<const thread_params *const>(pThreadParams));
        if (loopcount == n_frames - 1) {
            // wait for completion on final loop iteration
            fileWriterThread.join();
        } else {
            fileWriterThread.detach();
        }
#else
        char out_file[MAX_FILE_PATH_SIZE];

        snprintf(out_file, sizeof(out_file), "%s/%s_frame_%s_%05u.bin",
                 &folder_path[0], &frame_type[0], time_buffer, loopcount);
        std::ofstream rawFile(out_file, std::ios::out | std::ios::binary |
                                            std::ofstream::trunc);
        rawFile.write((const char *)&frameBuffer[0], frame_size);
        rawFile.close();

        if (frameBuffer != NULL) {
            free((void *)frameBuffer);
        }
        if (headerBuffer != NULL) {
            free((void *)headerBuffer);
        }
#endif
    } // End of for Loop

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> total_time = end_time - start_time;
    if (frame_type == "raw") {
        if (total_time.count() > 0.0) {
            double fps = (double)n_frames / total_time.count();
            LOG(INFO) << "FPS: " << fps;
        }
    }

    status = camera->stop();
    if (status != Status::OK) {
        LOG(INFO) << "Error stopping camera!";
    }
    return 0;
}

void fileWriterTask(const thread_params *const pThreadParams) {

    if (nullptr == pThreadParams) {
        return;
    }

    char out_file[MAX_FILE_PATH_SIZE] = {0};
    snprintf(out_file, sizeof(out_file), "%s/%s_frame_%s_%05" PRIu64 ".bin",
             pThreadParams->pFolderPath, pThreadParams->pFrame_type,
             pThreadParams->nFileTime, pThreadParams->nFrameCount);

    std::ofstream rawFile(out_file, std::ios::out | std::ios::binary |
                                        std::ofstream::trunc);
    rawFile.write((const char *)pThreadParams->pCaptureData,
                  pThreadParams->nTotalCaptureSize);
    rawFile.close();

    if (pThreadParams->pCaptureData != nullptr) {
        delete[] pThreadParams->pCaptureData;
    }
    if (pThreadParams->pHeaderData != nullptr) {
        delete[] pThreadParams->pHeaderData;
    }
    delete pThreadParams;
}