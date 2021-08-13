/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADICONTROLLER_H
#define ADICONTROLLER_H

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <aditof/depth_sensor_interface.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>

#include "ADIToFRecorder.h"
#include "safequeue.h"

namespace adicontroller
{

	class ADIController
	{
	public:
		/**
		* @brief	Constructor. Creates instance of main frame pointer and the first camera in the bus
		*/
		ADIController(const std::string &cameraIP = "");

		/**
		* @brief	Controller destructor
		*/
		~ADIController();

		/**
		* @brief	Start capture thread
		*/
		void StartCapture();

		/**
		* @brief	Stops capture thread
		*/
		void StopCapture();

		/**
		* @brief			  Set the camera index for active camera
		* @param cameraIndex	Index of camera in SDK system camera list
		*/
		void setCamera(const int cameraIndex) { m_cameraInUse = cameraIndex; }

		/**
		* @brief	Deprecated. There is another getMode from SDK.
		*			We may keep this in case needed for future
		*			imagers.
		*/
		std::string getMode() const;

		/**
		* @brief			Set the camera mode, directly to SDK.
		* @param	mode	Name of mode declared on SDK
		*/
		void setMode(const std::string &mode);

		/**
		* @brief	Fetch all supported modes. The information comes directly from SDK
		*/
		std::vector<std::string> getAvailableModes(std::vector<std::string>& availableModes);

		/**
		* @brief	Get camera temperature
		*/
		std::pair<float, float> getTemperature();

		aditof::Status writeAFEregister(uint16_t* address, uint16_t* data,
			uint16_t noOfEntries = 1);
		aditof::Status readAFEregister(uint16_t* address, uint16_t* data,
			uint16_t noOfEntries = 1);

		/**
		* @brief			Start recording video.
		* @param fileName	User sets a file name, along with its file extension
		* @param height	Height in pixels, gathered from current camera mode
		* @param width	Width in pixels, gathered from curren camera mode
		* @param fps		Frames to be recorded (No parameter time at this moment)
		*/
		void startRecording(const std::string& fileName, unsigned int height,
			unsigned int width, unsigned int fps);
		
		/**
		* @brief	Stops current recording
		*/
		void stopRecording();

		/**
		* @brief				Opens a currently saved recording and plays it back
		* @param	fileName	Chosen recording file name
		* @param	fps			Number of frames from file
		*/
		int startPlayback(const std::string& fileName, int& fps);		
		//int startPlayback(const std::string& fileName, int& fps, FSFStreamEnable& streamEnable);
		
		/**
		* @brief	Stops current playback recording
		*/
		void stopPlayback();

		/**
		* @brief Flag indicating that playback has finished.
		*		 It is used to stop playback thread.
		*/
		bool playbackFinished() const;

		/**
		* @brief Flag indicating that current recording is
		*        finished. It is used to stop recording
		*        thread.
		*/
		bool recordingFinished() const;

		/**
		* @brief Flag indicating that current video has
		*        been paused.
		*/
		bool playbackPaused() const;

		/**
		* @brief Pause current video playback
		*/
		void pausePlayback(bool paused) const;

		/**
		* @brief Gets the first or next frame.
		*/
		std::shared_ptr<aditof::Frame> getFrame();
		
		/**
		* @brief Requesting the SDK for a frame.
		*/
		void requestFrame();

		/**
		* @brief Flag indicating the existance of any
		*        camera in the system
		*/
		bool hasCamera() const;

		//int getRange() const;

		/**
		* @brief Get the maximum camera range.
		*/
		int getRangeMax() const;

		/**
		* @brief Get the minimum camera range.
		*/
		int getRangeMin() const;

		/**
		* @brief Gets camera bit counts.
		*/
		int getbitCount() const;

		/**
		* @brief Starts the binary to FSF conversion
		*        of a Point Cloud stream. No AB or Depth.
		*/
		bool startPointCloudBinToFSFConversion(const std::string& fileName, int& frames, int& width, int& height);

		std::vector<std::shared_ptr<aditof::Camera>> m_cameras;
		aditof::System m_system;
		std::unique_ptr<ADIToFRecorder> m_recorder;
		bool panicStop = false;
		size_t panicCount = 0;

	private:
		/**
		* @brief Sets a thread while capturing camera frames.
		*/
		void captureFrames();

	private:
		int m_cameraInUse;
		std::thread m_workerThread;
		std::atomic<bool> m_stopFlag;
		SafeQueue<std::shared_ptr<aditof::Frame> > m_queue;
		std::mutex m_mutex;
		std::mutex m_requestMutex;
		std::condition_variable m_requestCv;
		bool m_frameRequested;
		std::shared_ptr<aditof::Frame> m_framePtr;
	};
}// namespace adicontroller
#endif
