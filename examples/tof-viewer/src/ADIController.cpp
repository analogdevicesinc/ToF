/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <ADIController.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <iostream>
#include <memory>

using namespace adicontroller;

ADIController::ADIController(
    std::vector<std::shared_ptr<aditof::Camera>> camerasList)
    : m_cameraInUse(-1), m_frameRequested(false) {

    m_recorder = std::make_unique<ADIToFRecorder>();
    m_cameras = camerasList;
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        //auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        m_framePtr = std::make_shared<aditof::Frame>();
    } else {
        LOG(WARNING) << "No cameras found!";
    }
}

ADIController::~ADIController() {
    if (m_cameraInUse == -1) {
        return;
    }
    StopCapture();
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();
}

void ADIController::StartCapture() {
    if (m_cameraInUse == -1) {
        return;
    }

    m_stopFlag = false;
    m_workerThread =
        std::thread(std::bind(&ADIController::captureFrames, this));
}

void ADIController::StopCapture() {
    if (m_cameraInUse == -1) {
        return;
    }
    std::unique_lock<std::mutex> lock(m_requestMutex);
    m_stopFlag = true;
    m_cameras[m_cameraInUse]->stop();
    lock.unlock();
    m_requestCv.notify_one();
    if (m_workerThread.joinable()) {
        m_workerThread.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    m_queue.erase();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

std::string ADIController::getMode() const {
    // TODO: implement get mode
    return "";
}

void ADIController::setMode(const uint8_t &mode) {
    if (m_cameraInUse == -1) {
        return;
    }
    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    camera->setMode(mode);
}

std::pair<float, float> ADIController::getTemperature() {
    auto returnValue = std::make_pair<float, float>(0.0, 0.0);

    if (m_cameraInUse == -1) {
        return returnValue;
    }

    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];

    // TODO: Implement
    //std::shared_ptr<aditof::DepthSensorInterface> device = camera->getSensor();
    //device->readLaserTemp(returnValue.second);
    return returnValue;
}

void ADIController::startRecording(const std::string &fileName,
                                   unsigned int height, unsigned int width,
                                   unsigned int fps) {

    if (m_recorder != nullptr) {
        m_recorder = std::make_unique<ADIToFRecorder>();
    }
    m_recorder->setSaveBinaryFormat(this->m_saveBinaryFormat);
    m_recorder->startRecording(fileName, height, width, fps);
}

void ADIController::stopRecording() { m_recorder->stopRecording(); }

int ADIController::startPlayback(const std::string &fileName, int &fps) {
    return m_recorder->startPlayback(fileName, fps);
}

void ADIController::stopPlayback() { m_recorder->stopPlayback(); }

bool ADIController::playbackFinished() const {
    return m_recorder->isPlaybackFinished();
}

bool ADIController::recordingFinished() const {
    return m_recorder->isRecordingFinished();
}

bool ADIController::playbackPaused() const {
    return m_recorder->getPlaybackPaused();
}

void ADIController::pausePlayback(bool paused) const {
    m_recorder->setPlaybackPaused(paused);
}

std::shared_ptr<aditof::Frame> ADIController::getFrame() {
    if (m_recorder->isPlaybackEnabled()) {
        return m_recorder->readNewFrame();
    }
    return m_queue.dequeue();
}

void ADIController::requestFrame() {
    if (m_recorder->isPlaybackEnabled()) {
        m_recorder->requestFrame();
    } else {
        std::unique_lock<std::mutex> lock(m_requestMutex);
        m_frameRequested = true;
        lock.unlock();
        m_requestCv.notify_one();
    }
}

bool ADIController::hasCamera() const { return !m_cameras.empty(); }

void ADIController::captureFrames() {
    while (!m_stopFlag.load()) {
        std::unique_lock<std::mutex> lock(m_requestMutex);
        m_requestCv.wait(lock, [&] { return m_frameRequested || m_stopFlag; });

        if (m_stopFlag) {
            panicCount = 0;
            break;
        }

        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        auto frame = std::make_shared<aditof::Frame>();
        aditof::Status status = camera->requestFrame(frame.get());
        if (status != aditof::Status::OK) {
            if (panicCount >= 7) {
                panicStop = true;
            }

            m_queue.enqueue(frame);
            m_frameRequested = false;
            panicCount++;
            LOG(INFO) << "Trying to request frame... ";
            continue;
        }

        if (m_recorder->isRecordingEnabled()) {
            m_recorder->recordNewFrame(frame);
        }

        m_queue.enqueue(frame);
        m_frameRequested = false;
    }
}

//int ADIController::getRange() const
//{
//	aditof::CameraDetails cameraDetails;
//	m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
//		cameraDetails);
//	return cameraDetails.range;
//}

int ADIController::getRangeMax() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.maxDepth;
}

int ADIController::getRangeMin() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.minDepth;
}

int ADIController::getbitCount() const {
    aditof::CameraDetails cameraDetails;
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
        cameraDetails);
    return cameraDetails.bitCount;
}

int ADIController::getCameraInUse() const { return m_cameraInUse; }
