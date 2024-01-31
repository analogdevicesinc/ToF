/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "ADIToFRecorder.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#include <cstring>
#endif

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#define EMBED_HDR_LENGTH 128

ADIToFRecorder::ADIToFRecorder()
    : m_frameDetails{"", {}, "", 0, 0, 0, false}, m_recordTreadStop(true),
      m_playbackThreadStop(true), m_shouldReadNewFrame(true),
      m_playBackEofReached(false), m_numberOfFrames(0) {}

ADIToFRecorder::~ADIToFRecorder() {
    if (m_playbackFile.is_open()) {
        stopPlayback();
    }
    if (m_recordThread.joinable()) {
        m_recordThread.join();
    }
}

void ADIToFRecorder::createBinaryDirectory(std::string fileName) {
    auto pos = fileName.find(".raw");
    //Create a directory with same name as .raw file for .bin generation
    std::string rawFileDirectory = fileName;

    rawFileDirectory.erase(pos, std::string::npos);
    rawFileDirectory = rawFileDirectory + "_RAW";

#ifdef _WIN32
    static const int max_path = 260;
    char dir_path[max_path];
    // Create folder if not created already
    if (GetFullPathName(rawFileDirectory.c_str(), max_path, &dir_path[0],
                        NULL) == 0) {
        LOG(ERROR) << "Error Unable to open bin file directory. Error:"
                   << GetLastError();
    }
    BOOL dirResult = CreateDirectory(dir_path, NULL);
    if (dirResult || ERROR_ALREADY_EXISTS == GetLastError()) {
        m_fileNameRaw = rawFileDirectory + "/" + m_fileNameRaw;
    }
#else
    int err = mkdir(rawFileDirectory.c_str(), 0777);
    if (err < 0) {
        LOG(ERROR) << "Unable to create bin file directory";
    } else {
        m_fileNameRaw = rawFileDirectory + "/" + m_fileNameRaw;
    }
#endif
}

void ADIToFRecorder::startRecordingRaw(const std::string &fileName,
                                       unsigned int height, unsigned int width,
                                       unsigned int fps) {
    // m_fileNameRaw = "raw_frame_#.bin";
    // //save for binary
    // if (m_saveBinaryFormat)
    //     createBinaryDirectory(fileName);

    framesToRecord = fps;
    m_frameDetails.height = height;
    m_frameDetails.width = width;

    LOG(INFO) << "Recording...";
    m_finishRecording = false;
    m_recordTreadStop = false;
    m_recordThread =
        std::thread(std::bind(&ADIToFRecorder::recordThread, this, fileName));
}

void ADIToFRecorder::startRecording(const std::string &fileName,
                                    unsigned int height, unsigned int width,
                                    unsigned int fps) {
    if (fileName.find(".raw") != std::string::npos) {
        startRecordingRaw(fileName, height, width, fps);
    } else {
        LOG(ERROR) << "File format not supported";
        return;
    }
}

int ADIToFRecorder::findDigits(int number) {
    int numOfDigits = 0;

    do {
        numOfDigits++;
        number /= 10;
    } while (number);

    return numOfDigits;
}

void ADIToFRecorder::stopRecording() {
    m_recordTreadStop = true;
    if (m_recordThread.joinable()) {
        m_recordThread.join();
    }

    LOG(INFO) << "Recording has been completed.";
}

int ADIToFRecorder::startPlaybackRaw(const std::string &fileName, int &fps) {
    unsigned int height = 0;
    unsigned int width = 0;
    totalBits = 0;
    m_sizeOfHeader = METADATA_SIZE;

    m_playbackFile.open(fileName, std::ios::binary);

    if (!m_playbackFile) {
        LOG(ERROR) << "Failed open file!";
        return 0;
    }

    m_playbackFile.seekg(0, std::ios_base::end);
    fileSize = m_playbackFile.tellg();
    m_playbackFile.seekg(0, std::ios_base::beg);

    if (m_playbackFile.eof()) {
        LOG(WARNING) << "End of file reached! No more frames left to read.";
        m_playbackFile.close();
        return 0;
    }

    m_playbackFile.read(reinterpret_cast<char *>(&m_metadataStruct),
                        sizeof(aditof::Metadata));

    height = m_metadataStruct.height;
    width = m_metadataStruct.width;
    m_frameDetails.width = width;

    if (m_metadataStruct.bitsInDepth)
        totalBits += 2;
    if (m_metadataStruct.bitsInAb)
        totalBits += 2;
    if (m_metadataStruct.bitsInConfidence)
        totalBits += 4;
    if (m_metadataStruct.xyzEnabled == 1)
        totalBits += 6;

    // when the first frame is not carrying the correct metadata, skip it
    if (height == 0) {
        LOG(WARNING) << "first frame metadata not valid, first frame "
                        "skipped...";
        m_playbackFile.seekg(0, std::ios_base::beg);
        m_playbackFile.seekg(sizeof(uint16_t) * width * width * totalBits + 128,
                             std::ios_base::cur);
        m_playbackFile.read(reinterpret_cast<char *>(&m_metadataStruct),
                            sizeof(aditof::Metadata));

        height = m_metadataStruct.height;
        width = m_metadataStruct.width;
    }

    int sizeOfFrame = height * width * totalBits;
    currentPBPos = m_sizeOfHeader;
    m_numberOfFrames = fileSize / (m_sizeOfHeader + sizeOfFrame);

    m_frameDetails.height = height;
    m_frameDetails.width = width;

    m_playbackThreadStop = false;
    m_playBackEofReached = false;
    LOG(INFO) << "Streaming using raw or bin format. ";
    m_playbackThread =
        std::thread(std::bind(&ADIToFRecorder::playbackThread, this));

    return m_numberOfFrames;
}

int ADIToFRecorder::startPlayback(const std::string &fileName, int &fps) {
    if (fileName.find(".raw") != std::string::npos) {
        return startPlaybackRaw(fileName, fps);
    } else {
        LOG(ERROR) << "Selected Playback format not supported.";
        return 0;
    }
}

void ADIToFRecorder::stopPlayback() {
    m_playbackThreadStop = true;
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
    if (m_playbackThread.joinable()) {
        m_playbackThread.join();
    }
    clearVariables();
    m_playbackFile.close();
}

void ADIToFRecorder::recordNewFrame(std::shared_ptr<aditof::Frame> frame) {
    m_recordQueue.enqueue(frame);
}

std::shared_ptr<aditof::Frame> ADIToFRecorder::readNewFrame() {
    return m_playbackQueue.dequeue();
}

void ADIToFRecorder::requestFrame() {
    std::unique_lock<std::mutex> lock(m_playbackMutex);
    m_shouldReadNewFrame = true;
    lock.unlock();
    m_playbackCv.notify_one();
}

bool ADIToFRecorder::isRecordingEnabled() const { return !m_recordTreadStop; }

bool ADIToFRecorder::isPlaybackEnabled() const { return !m_playbackThreadStop; }

bool ADIToFRecorder::isPlaybackFinished() const { return m_playBackEofReached; }

bool ADIToFRecorder::isRecordingFinished() const { return m_finishRecording; }

bool ADIToFRecorder::isPlaybackPaused() { return isPaused; }

void ADIToFRecorder::setPlaybackPaused(bool paused) { isPaused = paused; }

int ADIToFRecorder::getNumberOfFrames() const { return m_numberOfFrames; }

void ADIToFRecorder::recordThread(const std::string fileName) {
    clock_t recording_start = clock();
    aditof::FrameHandler frameSaver;
    frameSaver.setOutputFilePath(std::string(""));
    while (!m_recordTreadStop) {
        int checkTime = (clock() - recording_start) / CLOCKS_PER_SEC;

        if (m_recordQueue.empty()) {
            continue;
        }
        if (frameCtr /*checkTime*/ >= framesToRecord) {
            frameCtr = 0; //reset.
            m_finishRecording = true;
            break;
        }

        auto frame = m_recordQueue.dequeue();

        uint16_t *header;
        frame->getData("metadata", &header);

        int width = m_frameDetails.width;
        int height = m_frameDetails.height;
        int captures = m_frameDetails.totalCaptures;
        int size = static_cast<int>(sizeof(uint16_t) * width * height);

        frameSaver.saveFrameToFile(*frame, fileName);

        // Create a new .bin file for each frame with raw sensor data
        // TODO: need to be implemented and exposed to UI
        // if (m_saveBinaryFormat) {
        //     std::string fileNameRawFrame = m_fileNameRaw;
        //     fileNameRawFrame.replace(fileNameRawFrame.find_last_of("#"), 1,
        //                              std::to_string(frameCtr));

        //     std::ofstream recordFileRaw;
        //     recordFileRaw.open(fileNameRawFrame,
        //                        std::ios::binary | std::ofstream::trunc);
        //     recordFileRaw.close();
        // }
        frameCtr++;
    }
}

void ADIToFRecorder::playbackThread() {
    int sizeOfFrame = 0;
    while (!m_playbackThreadStop) {

        if (!m_playbackFile.is_open()) {
            break;
        }
        if (m_playbackThreadStop) {
            break;
        }

        std::unique_lock<std::mutex> lock(m_playbackMutex);
        auto frame = std::make_shared<aditof::Frame>();
        aditof::FrameDataDetails dataDetails;

        m_playbackCv.wait(lock, [&]() { return m_shouldReadNewFrame; });
        m_shouldReadNewFrame = false;
        m_frameDetails.dataDetails.clear();

        dataDetails.type = "metadata";
        dataDetails.width = 1;
        dataDetails.height = EMBED_HDR_LENGTH;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "depth";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "ab";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "conf";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "xyz";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);

        frame->setDetails(m_frameDetails);
        frame->getData("metadata", &frameDataLocationHeader);
        frame->getData("depth", &frameDataLocationDEPTH);
        frame->getData("ab", &frameDataLocationAB);
        frame->getData("conf", &frameDataLocationCONF);
        frame->getData("xyz", &frameDataLocationXYZ);

        unsigned int width = m_frameDetails.width;
        unsigned int height = m_frameDetails.height;

        sizeOfFrame = totalBits * height * width;
        int size = static_cast<int>(sizeof(uint16_t) * width * height);

        if (m_playbackFile.eof() && currentPBPos >= fileSize) {
            LOG(WARNING) << "eof";
            memset(frameDataLocationAB, 0, sizeof(uint16_t) * width * height);
            m_playBackEofReached = true;
        } else {
            if (!isPaused && (currentPBPos < (fileSize - sizeOfFrame))) {
                m_playbackFile.seekg(currentPBPos, std::ios_base::beg);
                if (m_metadataStruct.bitsInDepth)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationDEPTH), size);
                if (m_metadataStruct.bitsInAb)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationAB), size);
                if (m_metadataStruct.bitsInConfidence)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationCONF),
                        size * 2);
                if (m_metadataStruct.xyzEnabled)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationXYZ),
                        size * 3);
                currentPBPos += (sizeOfFrame + EMBED_HDR_LENGTH);
            } else {
                m_playbackFile.seekg(currentPBPos, std::ios_base::beg);
                if (m_metadataStruct.bitsInDepth)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationDEPTH), size);
                if (m_metadataStruct.bitsInAb)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationAB), size);
                if (m_metadataStruct.bitsInConfidence)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationCONF),
                        size * 2);
                if (m_metadataStruct.xyzEnabled)
                    m_playbackFile.read(
                        reinterpret_cast<char *>(frameDataLocationXYZ),
                        size * 3);
            }
        }

        m_playbackQueue.enqueue(frame);
    }
}

void ADIToFRecorder::clearVariables() {
    //delete[] frameDataLocationAB;
    frameDataLocationAB = nullptr;
    frameDataLocationDEPTH = nullptr;
    frameDataLocationXYZ = nullptr;
    frameDataLocationCONF = nullptr;
    m_frameDetails.height = 0;
    m_frameDetails.passiveIRCaptured = false;
    m_frameDetails.cameraMode = "";
    m_frameDetails.totalCaptures = 0;
    m_frameDetails.type = "";
    m_frameDetails.width = 0;
    m_frameDetails.dataDetails.clear();
}
