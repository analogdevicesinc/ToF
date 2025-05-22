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
#include <chrono>

using namespace adicontroller;

ADIController::ADIController(
    std::vector<std::shared_ptr<aditof::Camera>> camerasList)
    : m_cameraInUse(-1), m_frameRequested(false) {

    m_cameras = camerasList;
    if (m_cameras.size()) {
        // Use the first camera that is found
        m_cameraInUse = 0;
        auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
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

    m_fps_startTime = std::chrono::system_clock::now();
    m_frame_counter = 0;
    m_stopFlag = false;
    m_workerThread = std::thread([this]() { captureFrames(); });
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

void ADIController::setMode(const uint8_t &mode) {
    if (m_cameraInUse == -1) {
        return;
    }
    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    camera->setMode(mode);
}

std::shared_ptr<aditof::Frame> ADIController::getFrame() {
    static std::shared_ptr<aditof::Frame> lastFrame = nullptr;
    static std::shared_ptr<aditof::Frame> frame;

    // Why do this?
	// The GUI is driven by frames being avaialble in the queue.
    // The last frame is resent until a frame becomes avaialble.
    // This keeps the GUI moving.
	if (m_queue.empty() && lastFrame != nullptr) {
		return lastFrame;
	}

    frame = m_queue.dequeue();

    lastFrame = frame;

    return frame;
}

bool ADIController::requestFrame() {
    std::unique_lock<std::mutex> lock(m_requestMutex, std::try_to_lock);

    if (!lock.owns_lock()) {
        return false;
    }

    m_frameRequested = true;
    lock.unlock();
    m_requestCv.notify_one();
    return true;
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
        auto fg = frame.get();
        aditof::Status status = camera->requestFrame(fg);
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

        m_frame_counter++;
        static uint32_t local_frame_counter;

        if (m_frame_counter == 0) {
            local_frame_counter = 0;
        }

        local_frame_counter++;
        auto currentTime = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = currentTime - m_fps_startTime;
        if (elapsed.count() >= 2) {
            m_framerate = static_cast<float>(local_frame_counter) / static_cast<float>(elapsed.count());
            local_frame_counter = 0;
            m_fps_startTime = currentTime;
        }

        m_queue.enqueue(frame);
        m_frameRequested = false;
    }
}

aditof::Status ADIController::getFrameRate(uint32_t &fps) {
    fps = static_cast<uint32_t>(std::round(m_framerate));

	return aditof::Status::OK;
}

aditof::Status ADIController::getFramesReceived(uint32_t& frames_recevied) {
    frames_recevied = static_cast<uint32_t>(m_frame_counter);

    return aditof::Status::OK;
}

aditof::Status ADIController::requestFrameOffline(uint32_t index) {

    if (m_stopFlag.load()) { 
        //PRB25
		//LOG(ERROR) << "Camera is stopped, cannot request frame.";
        //return aditof::Status::GENERIC_ERROR; 
    }

    std::unique_lock<std::mutex> lock(m_requestMutex);
    m_requestCv.wait(lock, [&] { return m_frameRequested || m_stopFlag; });

    auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
    auto frame = std::make_shared<aditof::Frame>();
    auto fg = frame.get();
    aditof::Status status = camera->requestFrame(fg, index);

    m_frame_counter++;

    m_queue.enqueue(frame);
    m_frameRequested = false;
}

int ADIController::getCameraInUse() const { return m_cameraInUse; }