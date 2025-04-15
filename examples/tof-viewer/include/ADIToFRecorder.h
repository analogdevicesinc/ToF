/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADITOFRECORDER_H
#define ADITOFRECORDER_H

#include "safequeue.h"
#include <aditof/frame.h>
#include <aditof/frame_definitions.h>
#include <aditof/frame_handler.h>
#include <atomic>
#include <fstream>
#include <functional>
#include <map>
#include <thread>

#ifdef _WIN32
#include <direct.h>
#endif

class ADIToFRecorder {

  public:
    /**
	* @brief ADI Recorder constructor
	*/
    ADIToFRecorder();

    /**
	* @brief ADI Recorder destructor
	*/
    ~ADIToFRecorder();

    /**
	* @brief			Start the recording thread
	* @param fileName	Given file name by the user
	* @param height		Image height, set by SDK
	* @param width		Image width, set by SDK
	* @param seconds	Number of seconds to record
	*/
    void startRecording(const std::string &fileName, unsigned int height,
                        unsigned int width, unsigned int seconds);

    /**
    * @brief           Start RAW fileformat recording
    * @param fileName	Given file name by the user
    * @param height		Image height, set by SDK
    * @param width		Image width, set by SDK
    * @param seconds	Number of seconds to record
    */
    void startRecordingRaw(const std::string &fileName, unsigned int height,
                           unsigned int width, unsigned int seconds);

    /**
	* @brief Stops Recording
	*/
    void stopRecording();

    /**
	* @brief			Starts raw playback
	* @param fileName	Given file name by the user
	* @param fps		Number of frames set by the user
	*/
    int startPlayback(const std::string &fileName);

    /**
	* @brief			Starts ADI's raw playback
	* @param fileName	Given file name by the user
	* @param fps		Number of frames set by the user
	*/
    int startPlaybackRaw(const std::string &fileName);

    /**
	* @brief Stop playback
	*/
    void stopPlayback();

    /**
	* @brief Recording new frame
	*/
    void recordNewFrame(std::shared_ptr<aditof::Frame> frame);

    /**
	* @brief Start reading new frame
	*/
    std::shared_ptr<aditof::Frame> readNewFrame();

    /**
	* @brief Request Frame
	*/
    void requestFrame();

    /**
	* @brief Gets Recording enabled flag
	*/
    bool isRecordingEnabled() const;

    /**
	* @brief Gets playback enabled flag
	*/
    bool isPlaybackEnabled() const;

    /**
	* @brief Gets playback EOF flag
	*/
    bool isPlaybackFinished() const;

    /**
	* @brief Gets recording finished flag
	*/
    bool isRecordingFinished() const;

    std::shared_ptr<aditof::Frame> playbackFrame;

    /**
	* @brief Record ADI's Raw Data
	*/
    void recordThread(const std::string fileName);

    /**
	* @brief ADIs RAW Playback Thread
	*/
    void playbackThread();

    /**
	* @brief Clears all playback and recording variables
	*/
    void clearVariables();

    /**
	* @brief Will build XYZ stream
	*/
    void processXYZData();

    void setPlaybackFrameNumber(const uint32_t frameNumber);
    uint32_t getPlaybackFrameNumber();

    size_t getCurrentPBPos() { return m_currentPBPos; }
    void setCurrentPBPos(size_t pos) { m_currentPBPos = pos; }

    bool getFinishRecording() { return m_finishRecording; }
    void setFinishRecording(bool finish) { m_finishRecording = finish; }

    aditof::FrameDetails getFrameDetails() { return m_frameDetails; }
    void setFrameDetails(aditof::FrameDetails frameDetails) {
        m_frameDetails = frameDetails;
    }

    bool getSaveBinaryFormat() { return m_saveBinaryFormat; }
    void setSaveBinaryFormat(bool saveBinary) {
        m_saveBinaryFormat = saveBinary;
    }

    uint32_t getBytesPerPixel() { return m_bytesPerPixel; }
    void setBytesPerPixel(uint32_t totalBytes) { m_bytesPerPixel = totalBytes; }

    uint32_t getNumberOfFrames() { return m_numberOfFrames; }
    void setNumberOfFrames(uint32_t numberOfFrames) {
        m_numberOfFrames = numberOfFrames;
    }

    bool getStopPlayback() { return m_stopPlayback; }
    void setStopPlayback(bool stopPlayback) { m_stopPlayback = stopPlayback; }

    uint16_t getSizeOfHeader() { return m_sizeOfHeader; }

    bool getPlaybackPaused() { return isPaused; }
    void setPlaybackPaused(bool paused = true) { isPaused = paused; }

    const uint32_t getFrameSize();

  private:
    /**
	* @brief Analyzes the given number and returns its number
	*        of digits.
	*/
    int findDigits(int number);

    /**
     * @brief                   Create binary directory
     * @param  fileDirectory    String with full path for saved raw file
     *
     */
    void createBinaryDirectory(std::string fileName);

  public:
    const uint16_t EMBED_HDR_LENGTH = 128; // Size in Bytes

  private:
    uint16_t *frameDataLocationDEPTH = nullptr;
    uint16_t *frameDataLocationAB = nullptr;
    uint16_t *frameDataLocationCONF = nullptr;
    uint16_t *frameDataLocationXYZ = nullptr;
    uint16_t *frameDataLocationHeader = nullptr;
    aditof::FrameDetails m_frameDetails;
    bool m_saveBinaryFormat = false;
    bool m_finishRecording = true;
    bool m_stopPlayback = false;
    uint32_t m_bytesPerPixel = 0;
    uint32_t m_numberOfFrames;
    size_t m_fileSize = 0;
    const uint16_t m_sizeOfHeader = EMBED_HDR_LENGTH;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_playbackQueue;
    SafeQueue<std::shared_ptr<aditof::Frame>> m_recordQueue;
    std::atomic<bool> m_playbackThreadStop;
    std::atomic<bool> m_recordTreadStop;
    std::condition_variable m_playbackCv;
    std::thread m_playbackThread;
    std::ifstream m_playbackFile;
    std::string m_fileNameRaw;
    std::thread m_recordThread;
    std::mutex m_playbackMutex;
    bool m_shouldReadNewFrame;
    bool m_playBackEofReached;
    bool isPaused = false;
    unsigned int m_secondsToRecord = 0;
    const int _depth = 2;
    const int _ab = 1;
    size_t frameCtr = 0;
    aditof::Metadata m_metadataStruct;
    size_t m_currentPBPos = 0;
};

#endif // ADITOFRECORDER_H
