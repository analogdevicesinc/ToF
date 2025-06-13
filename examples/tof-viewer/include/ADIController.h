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

#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <chrono>

#include "safequeue.h"

namespace adicontroller {

class ADIController {
  public:
    /**
		* @brief	Constructor. Creates instance of main frame pointer and the first camera in the bus
		*/
    ADIController(std::vector<std::shared_ptr<aditof::Camera>> camerasList);

    /**
		* @brief	Controller destructor
		*/
    ~ADIController();

    /**
		* @brief	Start capture thread
		*/
    void StartCapture(const uint32_t frameRate);

    /**
		* @brief	Stops capture thread
		*/
    void StopCapture();

    aditof::Status requestFrameOffline(uint32_t index);

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
    void setMode(const uint8_t &mode);

    /**
		* @brief	Get camera temperature
		*/
    std::pair<float, float> getTemperature();

    /**
		* @brief			Start recording video.
		* @param fileName	User sets a file name, along with its file extension
		* @param height	    Height in pixels, gathered from current camera mode
		* @param width	    Width in pixels, gathered from curren camera mode
		* @param seconds	Number of seconds to record
		*/
    void startRecording(const std::string &fileName, unsigned int height,
                        unsigned int width, unsigned int seconds);

    /**
		* @brief	Stops current recording
		*/
    void stopRecording();

    /**
		* @brief				Opens a currently saved recording and plays it back
		* @param	fileName	Chosen recording file name
		*/
    int startPlayback(const std::string &fileName);

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
    bool requestFrame();

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
    * @brief Gets camera in use.
    */
    int getCameraInUse() const;

    bool OutputDeltaTime(uint32_t frameNumber);

	aditof::Status getFrameRate(uint32_t& fps);
	aditof::Status getFramesReceived(uint32_t& framesRecevied);
    aditof::Status setPreviewRate(uint32_t frameRate, uint32_t previewRate = 1);
    aditof::Status getFramesLost(uint32_t& framesLost);
    
    std::vector<std::shared_ptr<aditof::Camera>> m_cameras;
    bool panicStop = false;
    size_t panicCount = 0;
    int m_cameraInUse;

  private:
    /**
		* @brief Sets a thread while capturing camera frames.
		*/
    void captureFrames();
    void calculateFrameLoss(const uint32_t frameNumber, uint32_t& prevFrameNumber, uint32_t& currentFrameNumber);
    bool shouldDropFrame(uint32_t frameNum);
    std::unordered_map<uint32_t, std::chrono::time_point<std::chrono::high_resolution_clock>> m_rxTimeLookUp;

  private:
    std::thread m_workerThread;
    std::atomic<bool> m_stopFlag;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_queue;
    std::mutex m_mutex;
    std::mutex m_requestMutex;
    std::condition_variable m_requestCv;
    bool m_frameRequested;
    std::shared_ptr<aditof::Frame> m_framePtr;
    float m_framerate = 0;
    uint32_t m_frame_counter;
    std::chrono::time_point<std::chrono::system_clock> m_fps_startTime;
    uint32_t m_preview_rate;
    uint32_t m_frame_rate;
    uint32_t m_frames_lost = 0;
    uint32_t m_prev_frame_number = -1;
    uint32_t m_current_frame_number = 0;
};
} // namespace adicontroller
#endif
