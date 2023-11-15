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
    if (m_recordFile.is_open()) {
        stopRecording();
    }
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
    m_fileNameRaw = "raw_frame_#.bin";

    //save for binary
    if (m_saveBinaryFormat)
        createBinaryDirectory(fileName);

    m_recordFile.open(fileName, std::ios::binary);
    m_recordFile.write(reinterpret_cast<const char *>(&height),
                       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&width),
                       sizeof(unsigned int));
    m_recordFile.write(reinterpret_cast<const char *>(&fps),
                       sizeof(unsigned int));
    framesToRecord = fps;
    m_frameDetails.height = height;
    m_frameDetails.width = width;

    LOG(INFO) << "Recording...";
    m_finishRecording = false;
    m_recordTreadStop = false;
    m_recordThread =
        std::thread(std::bind(&ADIToFRecorder::recordThread, this));
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

    m_recordFile.close();
    LOG(INFO) << "Recording has been completed.";
}

int ADIToFRecorder::startPlaybackRaw(const std::string &fileName, int &fps) {
    unsigned int height = 0;
    unsigned int width = 0;

    m_playbackFile.open(fileName, std::ios::binary);

    m_playbackFile.seekg(0, std::ios_base::end);
    fileSize = m_playbackFile.tellg();
    m_playbackFile.seekg(0, std::ios_base::beg);

    m_playbackFile.read(reinterpret_cast<char *>(&height), sizeof(int));
    m_playbackFile.read(reinterpret_cast<char *>(&width), sizeof(int));
    m_playbackFile.read(reinterpret_cast<char *>(&fps), sizeof(int));
    m_sizeOfHeader = 3 * sizeof(int);
    int sizeOfFrame = sizeof(uint16_t) * height * width;
    currentPBPos = m_sizeOfHeader;
    m_numberOfFrames = (fileSize - m_sizeOfHeader) / sizeOfFrame;

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

void ADIToFRecorder::recordThread() {
    clock_t recording_start = clock();
    while (!m_recordTreadStop) {
        int checkTime = (clock() - recording_start) / CLOCKS_PER_SEC;
        if (!m_recordFile.is_open()) {
            break;
        }

        if (m_recordQueue.empty()) {
            continue;
        }
        if (frameCtr /*checkTime*/ >= framesToRecord) {
            frameCtr = 0; //reset.
            m_finishRecording = true;
            break;
        }

        auto frame = m_recordQueue.dequeue();

        uint16_t *irData;
        frame->getData("ir", &irData);

        uint16_t *depthData;
        frame->getData("depth", &depthData);

        uint16_t *xyzData;
        frame->getData("xyz", &xyzData);

        uint16_t *rawData;
        frame->getData("raw", &rawData);

        uint16_t *header;
        frame->getData("metadata", &header);

        int width = m_frameDetails.width;
        int height = m_frameDetails.height;
        int captures = m_frameDetails.totalCaptures;
        int size = static_cast<int>(sizeof(uint16_t) * width * height);

        m_recordFile.write(reinterpret_cast<const char *>(irData), size);
        m_recordFile.write(reinterpret_cast<const char *>(depthData), size);
        m_recordFile.write(reinterpret_cast<const char *>(xyzData), size * 3);
        m_recordFile.write(reinterpret_cast<const char *>(header),
                           EMBED_HDR_LENGTH);

        //Create a new .bin file for each frame with raw sensor data
        if (m_saveBinaryFormat) {
            std::string fileNameRawFrame = m_fileNameRaw;
            fileNameRawFrame.replace(fileNameRawFrame.find_last_of("#"), 1,
                                     std::to_string(frameCtr));

            std::ofstream recordFileRaw;
            recordFileRaw.open(fileNameRawFrame,
                               std::ios::binary | std::ofstream::trunc);
            recordFileRaw.write(reinterpret_cast<const char *>(rawData),
                                static_cast<int>(captures * size));
            recordFileRaw.close();
        }
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

        dataDetails.type = "depth";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "ir";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "xyz";
        dataDetails.width = m_frameDetails.width;
        dataDetails.height = m_frameDetails.height;
        m_frameDetails.dataDetails.emplace_back(dataDetails);
        dataDetails.type = "metadata";
        dataDetails.width = 1;
        dataDetails.height = EMBED_HDR_LENGTH;
        m_frameDetails.dataDetails.emplace_back(dataDetails);

        frame->setDetails(m_frameDetails);
        frame->getData("ir", &frameDataLocationIR);
        frame->getData("depth", &frameDataLocationDEPTH);
        frame->getData("xyz", &frameDataLocationXYZ);
        frame->getData("metadata", &frameDataLocationHeader);

        unsigned int width = m_frameDetails.width;
        unsigned int height = m_frameDetails.height;

        sizeOfFrame = sizeof(uint16_t) * height * width;

        if (m_playbackFile.eof()) {
            memset(frameDataLocationIR, 0, sizeof(uint16_t) * width * height);
            m_playBackEofReached = true;
        } else {
            if (!isPaused && (currentPBPos < (fileSize - (sizeOfFrame)*10))) {
                int size = static_cast<int>(sizeof(uint16_t) * width * height);
                LOG(INFO) << "CURRENT BPOS " << currentPBPos;
                m_playbackFile.seekg(currentPBPos);
                currentPBPos += size * 5 + EMBED_HDR_LENGTH;
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationIR), size);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationDEPTH), size);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationXYZ), size * 3);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationHeader),
                    EMBED_HDR_LENGTH);

            } else {
                m_playbackFile.seekg(currentPBPos);
                int size = static_cast<int>(sizeof(uint16_t) * width * height);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationIR), size);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationDEPTH), size);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationXYZ), size * 3);
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationHeader),
                    EMBED_HDR_LENGTH);
            }
        }

        m_playbackQueue.enqueue(frame);
    }
}

void ADIToFRecorder::clearVariables() {
    //delete[] frameDataLocationIR;
    frameDataLocationIR = nullptr;
    frameDataLocationDEPTH = nullptr;
    frameDataLocationXYZ = nullptr;
    m_frameDetails.height = 0;
    m_frameDetails.passiveIRCaptured = false;
    m_frameDetails.cameraMode = "";
    m_frameDetails.totalCaptures = 0;
    m_frameDetails.type = "";
    m_frameDetails.width = 0;
    m_frameDetails.dataDetails.clear();
}
