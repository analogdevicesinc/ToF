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

void ADIToFRecorder::startRecordingRaw(const std::string &fileName,
                                       unsigned int height, unsigned int width,
                                       unsigned int seconds) {
    m_secondsToRecord = seconds;
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
                                    unsigned int seconds) {
    if (fileName.find(".bin") != std::string::npos) {
        startRecordingRaw(fileName, height, width, seconds);
    } else {
        LOG(ERROR) << "File format not supported";
        return;
    }
}

void ADIToFRecorder::stopRecording() {
    m_recordTreadStop = true;
    if (m_recordThread.joinable()) {
        m_recordThread.join();
    }

    LOG(INFO) << "Recording has been completed.";
}

int ADIToFRecorder::startPlaybackRaw(const std::string &fileName) {
    unsigned int height = 0;
    unsigned int width = 0;

    m_playbackFile.open(fileName, std::ios::binary);

    if (!m_playbackFile) {
        LOG(ERROR) << "Failed open file!";
        return 0;
    }

    m_playbackFile.seekg(0, std::ios_base::end);
    m_fileSize = m_playbackFile.tellg();
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

    // TODO: The sizes must be based on the settings from the metadata
    uint32_t depth_bytes = (16 / 8);
    uint32_t ab_bytes = (16 / 8);
    uint32_t conf_bytes = (32 / 8);
    uint32_t xyz_bytes = (48 / 8);

    m_bytesPerPixel = 0;
    m_bytesPerPixel += (m_metadataStruct.bitsInDepth) ? depth_bytes : 0;
    m_bytesPerPixel += (m_metadataStruct.bitsInAb) ? ab_bytes : 0;
    m_bytesPerPixel += (m_metadataStruct.bitsInConfidence) ? conf_bytes : 0;
    m_bytesPerPixel += (m_metadataStruct.xyzEnabled == 1) ? xyz_bytes : 0;

    // when the first frame is not carrying the correct metadata, skip it
    // Comment: If the first frame is corrupted, m_bytesPerPixel will be unknown or incorrect.
    //          In which case this will fail. To handle this return an error to the user
    //          by indicating 0 frames available.
    if (height == 0 || width == 0) {

        return 0;
#if 0  // See comment above.
        LOG(WARNING) << "first frame metadata not valid, first frame "
                        "skipped...";
        m_playbackFile.seekg(0, std::ios_base::beg);
        m_playbackFile.seekg(sizeof(uint16_t) * width * width *
                                     m_bytesPerPixel +
                                 EMBED_HDR_LENGTH,
                             std::ios_base::cur);
        m_playbackFile.read(reinterpret_cast<char *>(&m_metadataStruct),
                            sizeof(aditof::Metadata));

        height = m_metadataStruct.height;
        width = m_metadataStruct.width;
#endif // 0
    }

    m_frameDetails.height = height;
    m_frameDetails.width = width;

    m_playbackFile.seekg(0, std::ios_base::beg);
    m_currentPBPos = 0;

    m_numberOfFrames =
        m_fileSize / (m_sizeOfHeader + height * width * m_bytesPerPixel);

    m_playbackThreadStop = false;
    m_playBackEofReached = false;
    LOG(INFO) << "Streaming using bin format. ";
    m_playbackThread =
        std::thread(std::bind(&ADIToFRecorder::playbackThread, this));

    return m_numberOfFrames;
}

int ADIToFRecorder::startPlayback(const std::string &fileName) {
    if (fileName.find(".bin") != std::string::npos) {
        return startPlaybackRaw(fileName);
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

void ADIToFRecorder::recordThread(const std::string fileName) {

    clock_t recording_start = clock();
    aditof::FrameHandler frameSaver;
    frameSaver.setOutputFilePath(std::string(""));
    while (!m_recordTreadStop) {
        int checkTime = (clock() - recording_start) / CLOCKS_PER_SEC;

        if (m_recordQueue.empty()) {
            continue;
        }
        if (checkTime >= m_secondsToRecord) {
            LOG(INFO) << frameCtr << " frames recorded over " << m_secondsToRecord << "s.";
            frameCtr = 0; //reset.
            m_finishRecording = true;
            break;
        }

        auto frame = m_recordQueue.dequeue();

        frameSaver.saveFrameToFile(*frame, fileName);

        frameCtr++;
    }
}

void ADIToFRecorder::playbackThread() {
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

        uint32_t pixelArea = m_frameDetails.width * m_frameDetails.height;

        if (m_playbackFile.eof() && m_currentPBPos >= m_fileSize) {

            LOG(WARNING) << "eof";
            memset(frameDataLocationAB, 0, sizeof(uint16_t) * pixelArea);
            m_playBackEofReached = true;

        } else {

            size_t _currentPBPos =
                m_currentPBPos +
                static_cast<size_t>(getSizeOfHeader()); // Skip Header
            m_playbackFile.seekg(_currentPBPos, std::ios_base::beg);

            if (m_metadataStruct.bitsInDepth)
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationDEPTH),
                    pixelArea * 2);
            if (m_metadataStruct.bitsInAb)
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationAB),
                    pixelArea * 2);
            if (m_metadataStruct.bitsInConfidence)
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationCONF),
                    pixelArea * 4);
            if (m_metadataStruct.xyzEnabled)
                m_playbackFile.read(
                    reinterpret_cast<char *>(frameDataLocationXYZ),
                    pixelArea * 6);

            // FIX: there is no link to the frame rate, making this play as fast as possible, which is not desired.
            /*
            uint32_t sizeOfFrame = m_bytesPerPixel * pixelArea;
            if (!isPaused && (m_currentPBPos < (m_fileSize - sizeOfFrame))) { // Unpaused
                //Advance to the next frame
                m_currentPBPos += (sizeOfFrame + getSizeOfHeader());
            }
            */
        }

        m_playbackQueue.enqueue(frame);
    }
}

const uint32_t ADIToFRecorder::getFrameSize() {
    return (getFrameDetails().height * getFrameDetails().width *
                getBytesPerPixel() +
            getSizeOfHeader());
}

void ADIToFRecorder::setPlaybackFrameNumber(const uint32_t frameNumber) {
    m_currentPBPos = static_cast<size_t>(frameNumber * getFrameSize());
}

uint32_t ADIToFRecorder::getPlaybackFrameNumber() {
    return getCurrentPBPos() / getFrameSize();
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
