/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADITOFRECORDER_H
#define ADITOFRECORDER_H

#include <atomic>
#include <fstream>
#include <thread>
#include <map>
#include <functional>

#include <aditof/frame.h>
#include <fsf/fsf.h> 
#include <fsf/fsf_definitions.h>
#include <fsf_common.h>
#include "safequeue.h"
#include <glog/logging.h>


#ifdef _WIN32
#include <direct.h>
#endif 

#define FSF_CUSTOM_MAX_NUM_FRAMES 10000UL

typedef struct
{
	unsigned char* buffer;
	size_t size;
} fsf_data_t;

struct FSFStreamEnable
{
	bool active_br = true;
	bool depth = true;//This is Z
	bool unknown = false;
	bool raw = false;
	bool phase = false;
	bool x = true;
	bool y = true;
	bool rgb = false;
	bool raw_norm = false;
	bool raw_real = false;
	bool raw_imag = false;
	bool raw_real_filt = false;
	bool raw_imag_filt = false;
	bool radial = true;//This is radial Depth.
	bool radial_filt = false;
	bool common_mode = false;
	bool conf = false;
	bool variance = false;
	bool reflectivity = false;
};

class ADIToFRecorder
{
	
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
	* @biref			Start the recording thread
	* @param fileName	Given file name by the user
	* @param height		Image height, set by SDK
	* @param width		Image width, set by SDK
	* @param fps		Number of frames set by the user
	*/
	void startRecording(const std::string& fileName, unsigned int height,
		unsigned int width, unsigned int fps);

	void startRecordingRaw(const std::string& fileName, unsigned int height,
		unsigned int width, unsigned int fps); //ADI's Recording method

	/**
	* @brief Stops Recording
	*/
	void stopRecording();

	/**
	* @brief			Starts raw or FSF playback
	* @param fileName	Given file name by the user
	* @param fps		Number of frames set by the user
	*/
	int startPlayback(const std::string& fileName, int& fps);

	/**
	* @brief			Starts ADI's raw playback
	* @param fileName	Given file name by the user
	* @param fps		Number of frames set by the user
	*/
	int startPlaybackRaw(const std::string& fileName, int& fps);
	
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

	/**
	* @brief Gets playback paused flag
	*/
	bool isPlaybackPaused();

	/**
	* @brief Sets playback paused flag
	*/
	void setPlaybackPaused(bool paused = true);

	/**
	* @brief Gets number of frames
	*/
	int getNumberOfFrames() const;
	std::shared_ptr < aditof::Frame> playbackFrame;

	/**
	* @brief Record ADI's Raw Data
	*/
	void recordThread();

	/**
	* @brief ADIs RAW Playback Thread
	*/
	void playbackThread();

	/**
	* @brief When FSF Thread fails, reset varialbes
	*/
	void recordFSFThreadFail();

	uint16_t* frameDataLocationIR = nullptr;
	uint16_t* frameDataLocationDEPTH = nullptr;
	uint16_t* frameDataLocationXYZ = nullptr;
	bool _stopPlayback = false;

	//FSF Read:	
	/**
	* @brief Start FSF Playback
	*/
	void startPlaybackFSF();

	/**
	* @brief FSF Playback Thread
	*/
	void playbackFSFThread();	
	FSFStreamEnable _streamEnable;
	aditof::FileHeader fileHeader;
	aditof::FileHeader fileHeaderRaw;
	
	//FSF Write/Record
	/**
	* @brief			Start Recording FSF file format
	* @param fileName	Given file name by the user
	* @param fps		Number of frames set by the user
	* @param height		Image height, set by SDK
	* @param width		Image width, set by SDK	
	*/
	void startRecordingFSF(const std::string& fileName, int& fps, unsigned int height, unsigned int width);
	
	/**
	* @brief Record FSF Data Structure
	*/
	void recordFSFThread();
	bool isFSFRecording = false;
	bool fsfStop = false;

	/**
	* @brief Clears all playback and recording variables
	*/
	void clearVariables();

	/**
	* @brief Will build XYZ stream
	*/
	void processXYZData();

	/**
	* @brief Starts the binary to FSF conversion
	*        of a Point Cloud stream. No AB or Depth.
	*/
	bool startPointCloudBinToFSFConversion(const std::string& fileName, int& frames, int& width, int& height);

	int currentPBPos = 0;
	int m_numberOfFrames;
	int fileSize = 0;
	aditof::FrameDetails m_frameDetails;

	bool m_finishRecording = true;

private:
	SafeQueue<std::shared_ptr<aditof::Frame>> m_recordQueue;
	SafeQueue<std::shared_ptr<aditof::Frame>> m_playbackQueue;
	std::ofstream m_recordFile;
	//Raw stream
	std::string m_fileNameRaw;
	std::ifstream m_playbackFile;
	std::thread m_recordThread;
	std::thread m_playbackThread;
	std::atomic<bool> m_recordTreadStop;
	std::atomic<bool> m_playbackThreadStop;
	bool m_shouldReadNewFrame;
	std::mutex m_playbackMutex;
	std::condition_variable m_playbackCv;
	bool m_playBackEofReached;
	//bool m_finishRecording = true;
	bool isPaused = false;	
	int framesToRecord = 0;
	size_t frameCtr = 0;
	const int _ir = 1;
	const int _depth = 2;
	uint16_t irBuffer[sizeof(uint16_t)*1024*1024];

	//FSF Read:
	FILE* m_playbackFSFFile;
	fsf_data_t fsfData;
	aditof::FSF* pFsfRead = NULL;
	aditof::FSF* pFsfWrite = NULL;
	aditof::OptionalFileHeader optFileHeader;
	aditof::StreamInfo streamInfo;
	aditof::Stream stream;
	aditof::FileComment fileComment;
	std::string fsfFileName;
	std::map<uint32_t, int> streamTable;
	
	//FSF RAW in different document
	FILE* m_playbackFSFFileRaw;
	fsf_data_t fsfDataRaw;
	aditof::FSF* pFsfReadRaw = NULL;
	aditof::FSF* pFsfWriteRaw = NULL;
	aditof::OptionalFileHeader optFileHeaderRaw;
	aditof::StreamInfo streamInfoRaw;
	aditof::Stream streamRaw;
	aditof::FileComment fileCommentRaw;
	std::string fsfFileNameRaw;

	/**
	* @brief Creates FSF files for RAW_NORM Stream type
	*        and backwards compatibility for MS FSF format.
	*/
	void createRawFsfFile(std::string& fileName, unsigned int height, unsigned int width, uint8_t totalCaptures, uint32_t nFrames);
	
	/**
	* @brief Analyzes the given number and returns its number
	*        of digits.
	*/
	int findDigits(int number);
	size_t fsfFrameCtr = 0;

	/**
	* @brief Enables Stream types for FSF playback
	*/
	void enableStreamType(aditof::StreamType streamType);
};

#endif // ADITOFRECORDER_H
