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

void ADIController::StartCapture(const uint32_t frameRate) {
    if (m_cameraInUse == -1) {
        return;
    }

    m_fps_startTime = std::chrono::system_clock::now();
    m_frame_counter = 0;
    m_stopFlag = false;
    m_frames_lost = 0;
    m_preview_rate = 1;
    m_prev_frame_number = -1;
    m_current_frame_number = 0;
    setPreviewRate(frameRate, frameRate);
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

void ADIController::calculateFrameLoss(const uint32_t frameNumber, uint32_t &prevFrameNumber, uint32_t &currentFrameNumber) {
    
    // Do frame loss calculation.
    uint32_t frameNum = frameNumber;
    prevFrameNumber = currentFrameNumber;
    currentFrameNumber = frameNumber;

    if (currentFrameNumber - prevFrameNumber > 1) {
        m_frames_lost += (currentFrameNumber - prevFrameNumber - 1);
    }
}


void ADIController::captureFrames() {
    while (!m_stopFlag.load()) {

        if (m_preview_rate == 1) { // Allow the viewer to request frames as needed
            std::unique_lock<std::mutex> lock(m_requestMutex);
            m_requestCv.wait(lock, [&] { return m_frameRequested || m_stopFlag; });
        }

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
        if (elapsed.count() >= 5) {
            m_framerate = static_cast<float>(local_frame_counter) / static_cast<float>(elapsed.count());
            local_frame_counter = 0;
            m_fps_startTime = currentTime;
        }

        aditof::Metadata* metadata;
        status = frame->getData("metadata", (uint16_t**)&metadata);
        if (status == aditof::Status::OK && metadata != nullptr) {
            calculateFrameLoss(metadata->frameNumber, m_prev_frame_number, m_current_frame_number);
        }

		if (!shouldDropFrame(m_frame_counter)) {
            m_queue.enqueue(frame);
		}
        
        m_frameRequested = false;
    }
}

bool ADIController::shouldDropFrame(uint32_t frameNum) {
    if (m_frame_rate == 0) {
        m_frame_rate = 10; // Prevent error, fake frame rate.
        LOG(ERROR) << "m_frame_rate == 0 -> Using a default frame rate of " << m_frame_rate;
    }
    uint32_t out_idx_this = (frameNum * m_preview_rate) / m_frame_rate;
    uint32_t out_idx_next = ((frameNum + 1) * m_preview_rate) / m_frame_rate;
    return (out_idx_this == out_idx_next);
}

aditof::Status ADIController::getFramesLost(uint32_t& framesLost) {
    framesLost = m_frames_lost;

    return aditof::Status::OK;
}

aditof::Status ADIController::getFrameRate(uint32_t &fps) {
    fps = static_cast<uint32_t>(std::round(m_framerate));

	return aditof::Status::OK;
}

aditof::Status ADIController::getFramesReceived(uint32_t& framesRecevied) {
    framesRecevied = static_cast<uint32_t>(m_frame_counter);

    return aditof::Status::OK;
}

aditof::Status ADIController::setPreviewRate(uint32_t frameRate, uint32_t previewRate) {

	m_preview_rate = previewRate;
    m_frame_rate = frameRate;

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

    aditof::Metadata* metadata;
    status = frame->getData("metadata", (uint16_t**)&metadata);
    if (status == aditof::Status::OK && metadata != nullptr) {
        calculateFrameLoss(metadata->frameNumber, m_prev_frame_number, m_current_frame_number);
    }

    m_queue.enqueue(frame);
    m_frameRequested = false;
}

int ADIController::getCameraInUse() const { return m_cameraInUse; }