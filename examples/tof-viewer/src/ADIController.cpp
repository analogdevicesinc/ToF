/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <ADIController.h>
#include <glog/logging.h>
#include <iostream>

using namespace adicontroller;

ADIController::ADIController(const std::string &cameraIp) : m_cameraInUse(-1), m_frameRequested(false), m_recorder(new ADIToFRecorder())
{
	if (cameraIp.length() > 0) {
        m_system.getCameraListAtIp(m_cameras, cameraIp);
	} else {
        m_system.getCameraList(m_cameras);
	}
    if (m_cameras.size()) 
	{
        // Use the first camera that is found
        m_cameraInUse = 0;
//auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
        m_framePtr = std::make_shared<aditof::Frame>();
    }
	else 
	{
         LOG(WARNING) << "No cameras found!";
    }
}

ADIController::~ADIController()
{
    if (m_cameraInUse == -1) 
	{
        return;
    }
	StopCapture();
    m_cameras[static_cast<unsigned int>(m_cameraInUse)]->stop();	
}

void ADIController::StartCapture()
{
	if (m_cameraInUse == -1)
	{
		return;
	}

	m_stopFlag = false;
	m_workerThread = std::thread(std::bind(&ADIController::captureFrames, this));
}

void ADIController::StopCapture()
{
	if (m_cameraInUse == -1) 
	{
		return;
	}
	std::unique_lock<std::mutex> lock(m_requestMutex);
	m_stopFlag = true;
    m_cameras[m_cameraInUse]->stop();
	lock.unlock();
	m_requestCv.notify_one();
	if (m_workerThread.joinable()) 
	{
		m_workerThread.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
    m_queue.erase();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));        
}

std::string ADIController::getMode() const 
{
	// TODO: implement get mode
	return "";
}

void ADIController::setMode(const std::string &mode)
{
	if (m_cameraInUse == -1) 
	{
		return;
	}
	auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
	camera->setMode(mode);
//camera->setFrameType(mode);
}

std::vector<std::string> ADIController::getAvailableModes(std::vector<std::string>& availableModes)
{
	if (m_cameraInUse == -1)
	{
		return availableModes;
	}
	
	m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getAvailableModes(availableModes);
	return availableModes;
}

std::pair<float, float> ADIController::getTemperature() 
{
	auto returnValue = std::make_pair<float, float>(0.0, 0.0);

	if (m_cameraInUse == -1) 
	{
		return returnValue;
	}
	
	auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
	
	
	// TODO: Implement
	//std::shared_ptr<aditof::DepthSensorInterface> device = camera->getSensor();
	//device->readRegistersTemp(returnValue.first);
	//device->readLaserTemp(returnValue.second);
	return returnValue;
}

aditof::Status ADIController::writeAFEregister(uint16_t* address,
	uint16_t* data,
	uint16_t noOfEntries) 
{
	auto depthSensor =
		m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getSensor();
	return depthSensor->writeRegisters(address, data, noOfEntries);
}

aditof::Status ADIController::readAFEregister(uint16_t* address,
	uint16_t* data,
	uint16_t noOfEntries) 
{

	auto depthSensor = 
		m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getSensor();
	return depthSensor->readRegisters(address, data, noOfEntries);
}

void ADIController::startRecording(const std::string& fileName,
	unsigned int height,
	unsigned int width,
	unsigned int fps) 
{
	m_recorder->startRecording(fileName, height, width, fps);
}

void ADIController::stopRecording() { m_recorder->stopRecording(); }

int ADIController::startPlayback(const std::string& fileName, int& fps)
{
	return m_recorder->startPlayback(fileName, fps);
}

bool ADIController::startPointCloudBinToFSFConversion(const std::string& fileName, int& frames, int& width, int& height)
{
	return m_recorder->startPointCloudBinToFSFConversion(fileName, frames, width, height);
}

void ADIController::stopPlayback() { m_recorder->stopPlayback(); }

bool ADIController::playbackFinished() const 
{
	return m_recorder->isPlaybackFinished();
}

bool ADIController::recordingFinished() const
{
	return m_recorder->isRecordingFinished();
}

bool ADIController::playbackPaused() const
{
	return m_recorder->isPlaybackPaused();
}

void ADIController::pausePlayback( bool paused ) const
{
	m_recorder->setPlaybackPaused(paused);
}

std::shared_ptr<aditof::Frame> ADIController::getFrame() 
{
	if (m_recorder->isPlaybackEnabled()) 
	{
		return m_recorder->readNewFrame();
	}
	return m_queue.dequeue();
}

void ADIController::requestFrame() 
{
	if (m_recorder->isPlaybackEnabled()) 
	{
		m_recorder->requestFrame();
	}
	else 
	{
		std::unique_lock<std::mutex> lock(m_requestMutex);
		m_frameRequested = true;
		lock.unlock();
		m_requestCv.notify_one();
	}
}

bool ADIController::hasCamera() const { return !m_cameras.empty(); }

void ADIController::captureFrames() 
{
	while (!m_stopFlag.load()) 
	{
		std::unique_lock<std::mutex> lock(m_requestMutex);
		m_requestCv.wait(lock, [&] { return m_frameRequested || m_stopFlag; });

		if (m_stopFlag) 
		{
			panicCount = 0;
			break;
		}

		auto camera = m_cameras[static_cast<unsigned int>(m_cameraInUse)];
		auto frame = std::make_shared<aditof::Frame>();
		aditof::Status status = camera->requestFrame(frame.get());
		if (status != aditof::Status::OK) 
		{
			if (panicCount >= 10)
			{
				panicStop = true;
			}
			m_queue.enqueue(frame);
			m_frameRequested = false;			
			panicCount++;
			continue;
		}

		if (m_recorder->isRecordingEnabled()) 
		{
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

int ADIController::getRangeMax() const
{
	aditof::CameraDetails cameraDetails;
	m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
		cameraDetails);
	return cameraDetails.maxDepth;
}

int ADIController::getRangeMin() const
{
	aditof::CameraDetails cameraDetails;
	m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
		cameraDetails);
	return cameraDetails.minDepth;
}

int ADIController::getbitCount() const
{
	aditof::CameraDetails cameraDetails;
	m_cameras[static_cast<unsigned int>(m_cameraInUse)]->getDetails(
		cameraDetails);
	return cameraDetails.bitCount;
}
