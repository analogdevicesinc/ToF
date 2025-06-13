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
#include <unordered_map>
#include <cmath>

using namespace adicontroller;

#include <deque>
#include <cstddef>
class TrendWindow {
public:
    TrendWindow(size_t windowSize = 20) : maxSize(windowSize) {}

    void add(double value) {
        if (window.size() == maxSize) {
            window.pop_front();
        }
        window.push_back(value);
    }

    // Computes the slope of best-fit line (linear regression)
    // Returns 0 if not enough points
    double calculate_slope() const {
        if (window.size() < maxSize) return 0.0;

        size_t n = window.size();
        double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
        for (size_t i = 0; i < n; ++i) {
            sumX += i;
            sumY += window[i];
            sumXY += i * window[i];
            sumXX += i * i;
        }
        double denom = n * sumXX - sumX * sumX;
        if (denom == 0.0) return 0.0; // Avoid division by zero

        double m = (n * sumXY - sumX * sumY) / denom;
        return m;
    }

    // Helper to check if the trend is increasing
    bool isIncreasing(double threshold, double &slope) const {
        slope = calculate_slope();
        return slope > threshold;
    }

    void clear() {
        window.clear();
    }

private:
    size_t maxSize;
    std::deque<double> window;
};

static TrendWindow iw(100);

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

    m_rxTimeLookUp.clear();
    iw.clear();

    m_fps_startTime = std::chrono::system_clock::now();
    m_frame_counter = 0;
    m_stopFlag = false;
    m_frames_lost = 0;
    m_prev_frame_number = -1;
    m_current_frame_number = 0;
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
    return m_queue.empty()? nullptr : m_queue.dequeue();
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
            m_rxTimeLookUp[metadata->frameNumber] = std::chrono::high_resolution_clock::now();
            calculateFrameLoss(metadata->frameNumber, m_prev_frame_number, m_current_frame_number);
        }

		if (!shouldDropFrame(m_frame_counter)) {
            m_queue.enqueue(frame);
		}
        
        m_frameRequested = false;
    }
}

bool ADIController::OutputDeltaTime(uint32_t frameNumber) {
    auto it = m_rxTimeLookUp.find(frameNumber);
    if (it != m_rxTimeLookUp.end()) {
        long long duration;

        duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - m_rxTimeLookUp[frameNumber]).count();
        m_rxTimeLookUp.erase(frameNumber);
        iw.add(duration);

        double slope;

        bool ret = iw.isIncreasing(0.1, slope);

        return ret;
    }
    return false;
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

    return aditof::Status::OK;
}

int ADIController::getCameraInUse() const { return m_cameraInUse; }