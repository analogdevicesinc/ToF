/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <docopt.h>
#include <fsf_common.h>
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#define MULTI_THREADED 1
#define DATA_COLLECT_VERSION "1.3.0"
#define EMBED_HDR_LENGTH 128

enum : uint16_t {
    MAX_FILE_PATH_SIZE = 512,
    FRAME_TYPE_LEN = 20,
};

using namespace aditof;

#ifdef _WIN32
int main(int argc, char *argv[]);
#endif

typedef struct fsf_params {
    uint32_t n_frames;
    bool raw_frames;
    std::vector<aditof::StreamInfo> stream_info;
    std::vector<aditof::Stream> streams;
    aditof::FSF *pFileHandle;
} fsf_params;

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
    fsf_params *pFsfParams;    
} thread_params;

static const char kUsagePublic[] =
    R"(Data Collect.
    Usage:
      data_collect FILE
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--ext_fsync <0|1>] [--fsf <0|1>] [--wt <warmup>] [--ccb FILE] FILE
      data_collect (-h | --help) 

    Arguments:
      FILE            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help          Show this screen.
      --f <folder>       Input folder to save data to. Max folder name size is 512. [default: ./]
      --n <ncapture>     Number of frames to capture. [default: 1]
      --m <mode>         Mode to capture data in. [default: 10]
      --ext_fsync <0|1>  External FSYNC [0: Internal 1: External] [default: 0]
      --fsf <0|1>        FSF file type [0: Disable 1: Enable] [default: 0]
      --wt <warmup>      Warmup Time (in seconds) before data capture [default: 0]
      --ccb <FILE>       The path to store CCB content

    Valid mode (--m) options are:
        0: QMpixel short range (512x512) 
        3: Passive IR (1024x1024)
        5: 1Mpixel with passive IR (1024x1024)
        7: QMpixel (512x512)
       10: 1Mpixel (1024x1024)
)";
// Hide 'ft' (frame_type) option from public doc string
static const char kUsageInternal[] =
    R"(Data Collect.
    Usage:
      data_collect FILE
      data_collect [--f <folder>] [--n <ncapture>] [--m <mode>] [--ext_fsync <0|1>] [--ft <frame_type>] [--fsf <0|1>] [--wt <warmup>] [--ccb FILE] [--ip <ip>] [--fps <setfps>] FILE
      data_collect (-h | --help) 

    Arguments:
      FILE            Input config_default.json file (which has *.ccb and *.cfg)

    Options:
      -h --help          Show this screen.
      --f <folder>       Input folder to save data to. Max folder name size is 512. [default: ./]
      --n <ncapture>     Number of frames to capture. [default: 1]
      --m <mode>         Mode to capture data in. [default: 10]
      --ext_fsync <0|1>  External FSYNC [0: Internal 1: External] [default: 0]
      --ft <frame_type>  Type of frame to be captured [default: raw]
      --fsf <0|1>        FSF file type [0: Disable 1: Enable] [default: 0]
      --wt <warmup>      Warmup Time (in seconds) before data capture [default: 0]
      --ccb <FILE>       The path to store CCB content      
      --ip <ip>          Camera IP      
      --fps <setfps>     Set target FPS value [range: 50 to 200] 
)";

#ifdef MULTI_THREADED 
void fileWriterTask( const thread_params * const pThreadParams );
#endif

/**
 * @brief The function which initializes the fsf parameters if the fsf flag is raised high for generating FSF files.
 * @param Fsfparams: The fsf structure parameters to store the required information of the respective frame to fsf stream
 * @return null
 */
static aditof::FsfStatus fsf_initialize(fsf_params * const Fsfparams, const char *fileName) {
    aditof::FsfStatus fsfStatus = aditof::FsfStatus::FILE_NOT_CREATED;
    Fsfparams->pFileHandle = new aditof::FSF_Common{aditof::FsfMode::WRITE};
    if (Fsfparams->pFileHandle) {
        fsfStatus = Fsfparams->pFileHandle->CreateFsfFile(fileName);
    }

    if (fsfStatus == aditof::FsfStatus::SUCCESS) {
        std::string comments = "ADI Data Collect";
        std::string optionalFileHdr = "DataCollectVersion=" DATA_COLLECT_VERSION ";";

        aditof::FileHeader fileHeader = {};
        fileHeader.OptionalFileHdrSize = static_cast<uint32_t>(optionalFileHdr.size());
        fileHeader.FileCommentSize = static_cast<uint32_t>(comments.size());
        fileHeader.nFrames = Fsfparams->n_frames;
        fileHeader.nStreams = static_cast<uint32_t>(Fsfparams->streams.size());
        fsfStatus = Fsfparams->pFileHandle->SetFileHeader(fileHeader);

        Fsfparams->pFileHandle->SetFileComment(comments);      
        Fsfparams->pFileHandle->SetOptionalFileHeader(optionalFileHdr);        
    }
    return fsfStatus;
}

/**
 * @brief helper function to set fsf parameters for Stream Info of the respective frame.
 * @param Fsfparams: The fsf structure parameters to store the required information of the respective frame to fsf stream
 * @param fDetails: The frame details acquired from getDetails() function to fetch the height, width and subFrames of the frame.
 * @return null
 */
static aditof::FsfStatus fsf_setparameters(fsf_params *Fsfparams, FrameDetails *fDetails) {
    aditof::FsfStatus status = aditof::FsfStatus::SUCCESS;

    for(auto& info: Fsfparams->stream_info) {
        info.SystemID = 0;
        info.nRowsPerStream = fDetails->height;
        info.nColsPerStream = fDetails->width;
        info.BytesPerPixel = sizeof(int16_t);
        if (info.StreamType == static_cast<uint32_t>(StreamType::STREAM_TYPE_RAW_NORM) || 
            info.StreamType == static_cast<uint32_t>(StreamType::STREAM_TYPE_COMMON_MODE)) {
            info.ChannelFormat = static_cast<uint32_t>(ChannelFormat::FSF_CHANNEL_SIGNED_INT16);         
            info.OptionalStreamHdrSize = EMBED_HDR_LENGTH;
        }
        else {
            info.ChannelFormat = static_cast<uint32_t>(ChannelFormat::FSF_CHANNEL_UINT16);
            info.OptionalStreamHdrSize = 0; //TODO: debug stream header for depth frames
        }
    }

    for(std::size_t i = 0; i < Fsfparams->streams.size(); i++) {
        Fsfparams->streams[i].streamHeader.CompressedStreamSize = sizeof(uint16_t) * fDetails->height * fDetails->width;
        if (aditof::FsfStatus::SUCCESS != Fsfparams->pFileHandle->SetStreamInfo(static_cast<uint32_t>(i), Fsfparams->stream_info[i])) {
            status = aditof::FsfStatus::FAILED;
        }
    }

    return status;
}

/**
 * @brief The function to set the fsf file stream and generating FSF files.
 * @param pThreadParams: The thread parameters which stores all the required information of the respective frame
 * @return null
 */
static void fsf_setstream( const thread_params * const pThreadParams) {

    uint16_t *pData = pThreadParams->pCaptureData;   // Pointer to depth/raw data returned by getData
    uint8_t *pHeader = pThreadParams->pHeaderData;  // Pointer to frame headers returned by getData
    uint64_t loopcount = pThreadParams->nFrameCount; // The for loop index for requesting number of frames
    uint64_t frame_size = pThreadParams->pFramesize;

    if (pThreadParams->pFsfParams && pThreadParams->pFsfParams->pFileHandle) {
        for(std::size_t i = 0; i < pThreadParams->pFsfParams->streams.size(); i++) {
            uint16_t *pdata = pData + (frame_size * i);
            uint16_t *pdataEnd = pdata + frame_size;
            pThreadParams->pFsfParams->streams[i].streamHeader.TimeStamp = static_cast<uint32_t>(loopcount);
            pThreadParams->pFsfParams->streams[i].streamData.assign((char *)pdata, (char *)pdataEnd);
            pThreadParams->pFsfParams->streams[i].optionalStreamHeader.assign((char *)pHeader + (EMBED_HDR_LENGTH * i), EMBED_HDR_LENGTH);                
            pThreadParams->pFsfParams->pFileHandle->SetStream(static_cast<uint32_t>(loopcount), static_cast<uint32_t>(i), pThreadParams->pFsfParams->streams[i]);
        }
    }
}

/**
 * @brief helper function to save and close the fsf file successfully.
 * @param fileHandle: The fileHandle to an open fsf file.
 * @return null
 */
static void fsf_stop(fsf_params *Fsfparams) {
    if (Fsfparams->pFileHandle) {
        if (Fsfparams->pFileHandle->SaveFile() == aditof::FsfStatus::SUCCESS) {
            LOG(INFO) << "Fsf file saved";
        }
        if (Fsfparams->pFileHandle->CloseFile() == aditof::FsfStatus::SUCCESS) {
            LOG(INFO) << "Fsf file closed";
        }
    }
}

int main(int argc, char *argv[]) {

    char folder_path[MAX_FILE_PATH_SIZE];    // Path to store the raw/depth frames
    char json_file_path[MAX_FILE_PATH_SIZE]; // Get the .json file from command line
    std::string frame_type;         // Type of frame need to be captured (Raw/Depth/IR)
    
    uint32_t fsf_flag = false;
    fsf_params Fsfparams = {};

    uint16_t err = 0;
    uint32_t n_frames = 0;
    uint32_t mode = 0;
    uint32_t fps_counter = 0;
    uint32_t ext_frame_sync_en = 0;
    uint32_t warmup_time = 0;
    std::string ip;
    uint32_t setfps = 0;
    uint16_t fps_defaults[11] = {200, 105, 100, 200, 50, 50, 50, 105, 105, 50, 50};    

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    std::map<int, int> modeIndexMap = {{3, 0}, {5, 1}, {7, 2}, {10, 3}};

    std::map<std::string, docopt::value> args = docopt::docopt_private(kUsagePublic, kUsageInternal, {argv + 1, argv + argc}, true);

    // Parsing the arguments from command line
    err = snprintf(json_file_path, sizeof(json_file_path), "%s", args["FILE"].asString().c_str());
    if (err < 0) {
        LOG(ERROR) << "Error copying the json file path!";
        return 0;
    }

    // Parsing output folder
    if (args["--f"]) {
        err = snprintf(folder_path, sizeof(folder_path), "%s", args["--f"].asString().c_str());
    } else {
        err = snprintf(folder_path, sizeof(folder_path), "%s", ".");
    }

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
    if (args["--n"]) {
        n_frames = args["--n"].asLong();
    }
    Fsfparams.n_frames = n_frames;

    bool modeIsValid = false;
    // Parsing mode type
    if (args["--m"]) {
        mode = args["--m"].asLong();
        modeIsValid = (mode == 3 || mode == 5 || mode == 7 || mode == 10);
        if (!modeIsValid) {
            LOG(ERROR) << "Invalid Mode.." << mode << "The accepted values for mode are: 3, 5, 7, 10";
            return 0;
        }
    }

    // Parsing ip
    if (args["--ip"]) {
        ip = args["--ip"].asString();
        if (ip.empty()) {
            LOG(ERROR) << "Invalid ip (--ip) from command line!";
            return 0;
        }
    }    

    //set FPS value
    if (args["--fps"]) {
        setfps = args["--fps"].asLong();
        if (setfps > 200 || setfps < 50) {
            LOG(ERROR) << "Invalid FPS value.." << setfps;
            return 0;
        }
    }
    else {
        setfps = fps_defaults[mode];
    }    

    if (args["--ext_fsync"]) {
        ext_frame_sync_en = args["--ext_fsync"].asLong();
    }

    if (args["--ft"]) {
        frame_type = args["--ft"].asString();
    } else {
        frame_type = "raw";
    }
    if (frame_type.length() <= 0) {
        LOG(ERROR) << "Error parsing frame_type (--ft) from command line!";
        return 0;
    }    

    //Parsing Warm up time
    if (args["--wt"]) {
        warmup_time = args["--wt"].asLong();
        if (warmup_time < 0) {
            LOG(ERROR) << "Invalid warm up time input!";        
        }
    }

    // Checking fsf flag
    if (args["--fsf"]) {
        fsf_flag = args["--fsf"].asLong();
        if (fsf_flag && n_frames > DEFAULT_MAX_FRAMES) {
            LOG(ERROR) << "FSF file format is limited to a maximum of 300 frames!";
            return 0;
        }
    }

    //Parsing CCB path
    std::string ccbFilePath;
    if (args["--ccb"]) {
        ccbFilePath = args["--ccb"].asString();
    }

    LOG(INFO) << "Output folder: " << folder_path;
    LOG(INFO) << "Mode: " << mode;
    LOG(INFO) << "Number of frames: " << n_frames;
    LOG(INFO) << "Json file: " << json_file_path;
    LOG(INFO) << "Frame type is: " << frame_type;
    LOG(INFO) << "Warm Up Time is: " << warmup_time <<" seconds";
    
    if (!ip.empty()) {
        LOG(INFO) << "Ip address is: " << ip;
    }

    if (!ccbFilePath.empty()) {
        LOG(INFO) << "Path to store CCB content: " << ccbFilePath;
    }   

    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    if (ip.empty()) {
        system.getCameraList(cameras);
    } else {
        system.getCameraListAtIp(cameras, ip);
    }

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

    status = camera->setControl("powerUp", "call");
    if (status != Status::OK) {
        LOG(ERROR) << "Could not PowerUp camera!";
        return 0;
    }

    // optionally load configuration data from module memory
    status = camera->setControl("loadModuleData", "call");
    if (status != Status::OK) {
        LOG(INFO) << "No CCB/CFG data found in camera module,";
        LOG(INFO) << "Loading calibration(ccb) and configuration(cfg) data from JSON config file...";
    }

    // Get frame types
    std::vector<std::string> frameTypes;
    status = camera->getAvailableFrameTypes(frameTypes);
    if (status != Status::OK || frameTypes.empty()) {
        LOG(ERROR) << "Could not aquire frame types";
        return 0;
    }

    if (!modeIsValid) {
        LOG(ERROR) << "Camera mode: " << mode << "is incorrect. The accepted values for mode are: 3, 5, 7, 10";
        return 0;
    }  

    // Set UVC format type and camera frame details
    if ("raw" == frame_type) {
        camera->setControl("enableDepthCompute", "off");
        Fsfparams.raw_frames = true;
    } else if ("depth" == frame_type) {
        if (frameTypes[modeIndexMap[mode]] == "pcm") {
            LOG(ERROR) << frameTypes[modeIndexMap[mode]] << " mode doesn't contain depth data, please set --ft (frameType) to raw.";
            return 0;
        }
        else {
            camera->setControl("enableDepthCompute", "on");
            Fsfparams.raw_frames = false;
        }        
    }
    else {
        LOG(ERROR) << "unsupported frame type!";
        return 0;      
    }

    status = camera->setFrameType(frameTypes[modeIndexMap[mode]]);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }    

    char fsf_file[MAX_FILE_PATH_SIZE];  
    char time_buffer[128];
    time_t rawtime;
    time (&rawtime);
    struct tm timeinfo;
#ifdef _WIN32
    localtime_s(&timeinfo, &rawtime);
#else
    localtime_r(&rawtime, &timeinfo);
#endif
    strftime(time_buffer,sizeof(time_buffer),"%Y%m%d%H%M%S", &timeinfo);    
    if (fsf_flag) {
        err = snprintf(fsf_file, sizeof(fsf_file), "%s/%s_frames_%s.fsf", folder_path, frame_type.c_str(), time_buffer);
        if (err < 0) {
            LOG(ERROR) << "Could not create FSF file!";
            return 0;
        }

        CameraDetails camDetails;
        camera->getDetails(camDetails);
        int totalCaptures = camDetails.frameType.totalCaptures;

        if (Fsfparams.raw_frames) {
            for (int ix = 0; ix < totalCaptures; ++ix) {
                aditof::StreamInfo info = {};
                aditof::Stream stream = {};
                if (camDetails.frameType.passiveIRCaptured && ix == totalCaptures - 1) {
                    info.StreamType = static_cast<uint32_t>(StreamType::STREAM_TYPE_COMMON_MODE);
                }
                else {
                    info.StreamType = static_cast<uint32_t>(StreamType::STREAM_TYPE_RAW_NORM);
                }
                Fsfparams.stream_info.push_back(info);
                Fsfparams.streams.push_back(stream);
            }
        }
        else { //depth/AB
            LOG(ERROR) << "Depth FSF file not supported!";
            return 0;            
#if 0
            aditof::StreamInfo depth_info = {};
            aditof::Stream depth_stream = {};
            depth_info.StreamType = StreamType::STREAM_TYPE_DEPTH;
            Fsfparams.stream_info.push_back(depth_info);
            Fsfparams.streams.push_back(depth_stream);

            aditof::StreamInfo ab_info = {};
            aditof::Stream ab_stream = {};
            ab_info.StreamType = StreamType::STREAM_TYPE_ACTIVE_BR;
            Fsfparams.stream_info.push_back(ab_info);
            Fsfparams.streams.push_back(ab_stream);
#endif
        }

        if (FsfStatus::SUCCESS != fsf_initialize(&Fsfparams, fsf_file)) {
            LOG(ERROR) << "FSF file could not be created";
            return 0;            
        }
        LOG(INFO) << "FSF File name: " << fsf_file;  

        if (FsfStatus::SUCCESS != fsf_setparameters(&Fsfparams, &camDetails.frameType)) {
            LOG(ERROR) << "Error initializeing FSF file!";
            return 0;
        }
    }

#if 0
    camera->setControl("setFPS", std::to_string(setfps));
    if (status != Status::OK) {
        LOG(ERROR) << "Error setting camera FPS to " << setfps;
        return 0;
    }
#endif    

    // Program the camera with cfg passed, set the mode by writing to 0x200 and start the camera        
    status = camera->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start camera!";
        return 0;
    }

    if (ext_frame_sync_en == 0) {
        status = camera->setControl("syncMode", "0, 0"); // Master, timer driven

    } else if (ext_frame_sync_en == 1) {
        status = camera->setControl("syncMode", "2, 0"); // Slave, 1.8v  // TODO: This configuration is required by oFilm, expand for finer control
    }

    aditof::Frame frame;
    FrameDetails fDetails;
    std::string frameType;
    
    uint16_t *frameBuffer;
    uint8_t  *headerBuffer;
    uint32_t height;
    uint32_t width;
    uint32_t subFrames;
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
            elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(warmup_end - warmup_start).count();
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
        std::string attrVal;
        frame.getAttribute("total_captures", attrVal);
        subFrames = std::stoi(attrVal);
        frameType = "raw";

        // Depth Data
        if (frame_type == "depth") {
            frame_size = sizeof(uint16_t) * height * width;
            frameType = "depth";
        } else if (frame_type == "raw") {
            frame_size = sizeof(uint16_t) * height * width * subFrames;
            frameType = "frameData"; // TO DO: change this to "raw" when it gets done
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

#if 0   // TO DO: uncomment this one the header becomes available
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
        thread_params * pThreadParams = new thread_params();
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
        pThreadParams->pFsfParams = (fsf_flag) ? &Fsfparams : NULL;
        pThreadParams->pFramesize = height * width;

        /* fileWriterThread handles the copying of raw/depth frames to a file */
        std::thread fileWriterThread(fileWriterTask, const_cast<const thread_params * const>(pThreadParams));
        if (loopcount == n_frames -1) {
            // wait for completion on final loop iteration
            fileWriterThread.join();
        }
        else {
            fileWriterThread.detach();
        }
#else
        char out_file[MAX_FILE_PATH_SIZE];

        if (!fsf_flag) {
            snprintf(out_file, sizeof(out_file), "%s/%s_frame_%s_%05u.bin", &folder_path[0], &frame_type[0], time_buffer, loopcount);
            std::ofstream rawFile(out_file, std::ios::out | std::ios::binary | std::ofstream::trunc);
            rawFile.write((const char *)&frameBuffer[0], frame_size);
            rawFile.close();
        }
        else {
            for(std::size_t i = 0; i < Fsfparams.streams.size(); i++) {
                uint16_t *pdata = frameBuffer + (height*width * i);
                uint16_t *pdataEnd = pdata + height*width;
                uint8_t *pheader = headerBuffer + (EMBED_HDR_LENGTH * i);
                Fsfparams.streams[i].streamHeader.TimeStamp = static_cast<uint32_t>(loopcount);              
                Fsfparams.streams[i].streamData.assign((char *)pdata, (char *)pdataEnd);
                Fsfparams.streams[i].optionalStreamHeader.assign((char *)pheader, EMBED_HDR_LENGTH);                
                Fsfparams.pFileHandle->SetStream(loopcount, static_cast<uint32_t>(i), Fsfparams.streams[i]);
            }
        }

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

    // Store CCB to file
    if (!ccbFilePath.empty()) {
        status = camera->setControl("saveModuleCCB", ccbFilePath);
        if (status != Status::OK) {
            LOG(INFO) << "Failed to store CCB to " << ccbFilePath;
        }
    }

    fsf_stop(&Fsfparams);
    status = camera->stop();
    if (status != Status::OK) {
        LOG(INFO) << "Error stopping camera!";
    }
    return 0;
}

void fileWriterTask( const thread_params * const pThreadParams ) {

    if (nullptr == pThreadParams) {
        return;
    }

    char out_file[MAX_FILE_PATH_SIZE] = {0};
    if (nullptr == pThreadParams->pFsfParams) {
        snprintf(out_file, sizeof(out_file), "%s/%s_frame_%s_%05lld.bin", pThreadParams->pFolderPath, pThreadParams->pFrame_type, pThreadParams->nFileTime,
             pThreadParams->nFrameCount);

        std::ofstream rawFile(out_file, std::ios::out | std::ios::binary | std::ofstream::trunc );
        rawFile.write((const char *)pThreadParams->pCaptureData, pThreadParams->nTotalCaptureSize);
        rawFile.close();
    }
    else {
        fsf_setstream(pThreadParams);
    }

    if (pThreadParams->pCaptureData != nullptr) {
        delete [] pThreadParams->pCaptureData;
    }
    if (pThreadParams->pHeaderData != nullptr) {
        delete [] pThreadParams->pHeaderData;
    }    
    delete pThreadParams;
}

