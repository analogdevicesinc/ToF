/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "ADIToFRecorder.h"
#include <ADIBinToFSF.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#define EMBED_HDR_LENGTH 128

ADIToFRecorder::ADIToFRecorder()
	: m_frameDetails{ "", {}, "", 0, 0, 0, false}, m_recordTreadStop(true),
	m_playbackThreadStop(true), m_shouldReadNewFrame(true),
	m_playBackEofReached(false), m_numberOfFrames(0), fsfFileName(""), fsfFileNameRaw(""){}

ADIToFRecorder::~ADIToFRecorder() 
{
	if (m_recordFile.is_open()) 
	{
		stopRecording();
	}
	if (m_playbackFile.is_open()) 
	{
		stopPlayback();
	}
	if (m_recordThread.joinable())
	{
		m_recordThread.join();
	}
}

void ADIToFRecorder::startRecording(const std::string& fileName,
	unsigned int height, unsigned int width,
	unsigned int fps) 
{
	if (fileName.find(".fsf") != std::string::npos)
	{//This is an FSF file, then go to FSF Recording
		startRecordingFSF(fileName, (int&)fps, height, width);
		isFSFRecording = true;
	}
	else
	{
		m_fileNameRaw = "raw_frame_#.bin";
		auto pos = fileName.find(".raw");
		if (pos != std::string::npos)
		{
			//Create a directory with same name as .raw file for .bin generation
			std::string rawFileDirectory = fileName;
			rawFileDirectory.erase(pos, std::string::npos);
			rawFileDirectory = rawFileDirectory + "_RAW";        
#ifdef _WIN32
			static const int max_path = 260;
			char dir_path[max_path];    
			// Create folder if not created already
			if (GetFullPathName(rawFileDirectory.c_str(), max_path, &dir_path[0], NULL) == 0) {
				LOG(ERROR) << "Error Unable to open bin file directory. Error:" << GetLastError();
			}

			BOOL dirResult = CreateDirectory(dir_path, NULL);
			if (dirResult || ERROR_ALREADY_EXISTS == GetLastError()) {
				m_fileNameRaw = rawFileDirectory + "/" + m_fileNameRaw;
			}			
#else
			int err = mkdir(rawFileDirectory.c_str(), 0777);
			if (err < 0) {
				LOG(ERROR) << "Unable to create bin file directory";
			}
			else {
				m_fileNameRaw = rawFileDirectory + "/" + m_fileNameRaw;
			}
#endif

		}
		m_recordFile.open(fileName, std::ios::binary);
		m_recordFile.write(reinterpret_cast<const char*>(&height),
			sizeof(unsigned int));
		m_recordFile.write(reinterpret_cast<const char*>(&width),
			sizeof(unsigned int));
		m_recordFile.write(reinterpret_cast<const char*>(&fps),
			sizeof(unsigned int));
		isFSFRecording = false;
		framesToRecord = fps;
		m_frameDetails.height = height;
		m_frameDetails.width = width;		

		LOG(INFO) << "Recording...";
		m_finishRecording = false;
		m_recordTreadStop = false;
		m_recordThread =
			std::thread(std::bind(&ADIToFRecorder::recordThread, this));
	}
}


void ADIToFRecorder::startRecordingFSF(const std::string& fileName, int& fps, unsigned int height, unsigned int width)
{
	fsfFileName = fileName;
	fsfFileNameRaw = fileName;
	//Set files with different sufix
	if (fileName.find(".fsf") != std::string::npos)
	{
		//Making sure that original file name is .fsf format
		fsfFileNameRaw.replace(fsfFileNameRaw.find(".fsf"), 4, "");
		fsfFileNameRaw += "_RAW.fsf";
		fsfFileName.replace(fsfFileName.find(".fsf"), 4, "");
		fsfFileName += "_XYZ.fsf";
		//Now we have a different file extension
	}
	framesToRecord = fps;//FPS as opposed to be in seconds.
	//pFsfWrite = new FSF_Common{ FsfMode::WRITE };
	pFsfWrite = new aditof::FSF_Common{ aditof::FsfMode::WRITE, (const unsigned int)fps };
	pFsfWriteRaw = new aditof::FSF_Common{ aditof::FsfMode::WRITE, (const unsigned int)fps };
	if (pFsfWrite->CreateFsfFile(fsfFileName.c_str()) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file";
		return;
	}

	createRawFsfFile(fsfFileNameRaw, height, width, m_frameDetails.totalCaptures, (uint32_t)fps);
	

	//For FSF File extension
	//Create File Header
	fileHeader.OptionalFileHdrSize = 3;
	fileHeader.FileCommentSize = 0;
	fileHeader.nFrames = fps;// fps;
	int _numStreams = 0;
	if(_streamEnable.active_br)
		_numStreams++;
	if (_streamEnable.depth)//Z for point cloud
		_numStreams++;
	//if (_streamEnable.y && _streamEnable.x)//Point Cloud
	//	_numStreams++;
	if (_streamEnable.x)
		_numStreams++;
	if (_streamEnable.y)
		_numStreams++;
	if (_streamEnable.radial)//Depth
		_numStreams++;
	fileHeader.nStreams = _numStreams;//This variable can't be put after pFsfWrite->SetStreamInfo function. It will create an exception
	if (pFsfWrite->SetFileHeader(fileHeader) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error while creating file header.";
		return;
	}
	int _streamCounter = 0;
	
	//Clear Dictionary containing Stream index information
	streamTable.clear();
	
	if (_streamEnable.active_br)
	{
		//Stream Information IR	
		//Create Stream Info IR, streamIdx = 0
		streamInfo.SystemID = 0;
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR;
		streamInfo.ChannelFormat = (uint32_t)aditof::ChannelFormat::FSF_CHANNEL_UINT16;
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width; 
		streamInfo.OptionalStreamHdrSize = 3;
		streamInfo.StreamCommentSize = 2;
		streamInfo.CompressionScheme = 0;
		if (pFsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for ACTIVE_BR.";
			return;
		}
		streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR] = _streamCounter;
		_streamCounter++;
	}
	
	if (_streamEnable.radial)//Only Depth
	{
		//Stream Information DEPTH
		//Create Stream Info DEPTH, streamIdx = 1
		streamInfo.SystemID = 0;
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_RADIAL;
		streamInfo.ChannelFormat = (uint32_t)aditof::ChannelFormat::FSF_CHANNEL_SIGNED_INT16;
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;
		streamInfo.StreamCommentSize = 5;
		streamInfo.CompressionScheme = 0;
		if(pFsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_RADIAL.";
			return;
		}
		streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_RADIAL] = _streamCounter;
		_streamCounter++;		
	}
	
	if (_streamEnable.x)
	{
		//Stream Information X
		//Create Stream Info X, streamIdx = 2
		streamInfo.SystemID = 0;
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_X;
		streamInfo.ChannelFormat = (uint32_t)aditof::ChannelFormat::FSF_CHANNEL_SIGNED_INT16;
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;
		streamInfo.StreamCommentSize = 1;
		streamInfo.CompressionScheme = 0;
		if(pFsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_X.";
			return;
		}
		streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_X] = _streamCounter;
		_streamCounter++;
	}
	if (_streamEnable.y)
	{
		//Stream Information Y
		//Create Stream Info Y, streamIdx = 3
		streamInfo.SystemID = 0;
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_Y;
		streamInfo.ChannelFormat = (uint32_t)aditof::ChannelFormat::FSF_CHANNEL_SIGNED_INT16;
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;
		streamInfo.StreamCommentSize = 1;
		streamInfo.CompressionScheme = 0;
		if(pFsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_Y.";
			return;
		}
		streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_Y] = _streamCounter;
		_streamCounter++;
	}

	if (_streamEnable.depth)
	{
		//Stream Information Z
		//Create Stream Info Z, streamIdx = 4
		streamInfo.SystemID = 0;
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH;
		streamInfo.ChannelFormat = (uint32_t)aditof::ChannelFormat::FSF_CHANNEL_SIGNED_INT16;
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;
		streamInfo.StreamCommentSize = 1;
		streamInfo.CompressionScheme = 0;
		if (pFsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_DEPTH.";
			return;
		}
		streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH] = _streamCounter;
		_streamCounter++;
	}

	//Create Optional File header	
	optFileHeader = "ADI";//TODO: Make a real optional file header
	stream.optionalStreamHeader = optFileHeader;
	if (pFsfWrite->SetOptionalFileHeader(optFileHeader) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting optional header file.";
		return;
	}

	//File Comment
	fileComment = "";
	if (pFsfWrite->SetFileComment(fileComment) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting file comment.";
		return;
	}

	LOG(INFO) << "Recording...";
	m_finishRecording = false;
	m_recordTreadStop = false;
	m_recordThread =
		std::thread(std::bind(&ADIToFRecorder::recordFSFThread, this));
}

void ADIToFRecorder::createRawFsfFile(std::string& fileName, unsigned int height, unsigned int width, uint8_t totalCaptures, uint32_t nFrames)
{
	if (pFsfWriteRaw->CreateFsfFile(fileName.c_str()) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file for RAW captures";
		return;
	}

	/*****************************************/
	//Create RAW 
	//Create File Header
	fileHeaderRaw.OptionalFileHdrSize = 3;
	fileHeaderRaw.FileCommentSize = 0;
	fileHeaderRaw.nFrames = nFrames;// fps;
	//Make nStreams configurable depending on the current mode selected
	fileHeaderRaw.nStreams = totalCaptures;//This variable can't be put after pFsfWrite->SetStreamInfo function. It will create an exception
	if (pFsfWriteRaw->SetFileHeader(fileHeaderRaw) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create BIN file, error while creating file header.";
		return;
	}

	//Stream Information RAW
	//Create Stream Info RAW, streamIdx = 0
	streamInfoRaw.SystemID = 0;//Verify this concept
	streamInfoRaw.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_RAW_NORM;
	streamInfoRaw.ChannelFormat = static_cast<uint32_t>(aditof::ChannelFormat::FSF_CHANNEL_SIGNED_INT16);//Verify this concept
	streamInfoRaw.BytesPerPixel = 2;
	streamInfoRaw.nRowsPerStream = height;
	streamInfoRaw.nColsPerStream = width;
	streamInfoRaw.OptionalStreamHdrSize = EMBED_HDR_LENGTH;
	streamInfoRaw.StreamCommentSize = 3;//Verify this concept
	streamInfoRaw.CompressionScheme = 0;//verify this concept		
	for (size_t streamCtr = 0; streamCtr < totalCaptures; streamCtr++)
	{
		if (pFsfWriteRaw->SetStreamInfo(streamCtr, streamInfoRaw) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for STREAM_TYPE_RAW.";
			return;
		}
	}

	//Create Optional File header	
	optFileHeaderRaw = "ADI";//TODO: Make a real optional file header
	streamRaw.optionalStreamHeader = optFileHeaderRaw;
	if (pFsfWriteRaw->SetOptionalFileHeader(optFileHeaderRaw) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting optional header file.";
		return;
	}

	//File Comment
	fileComment = "";
	if (pFsfWriteRaw->SetFileComment(fileComment) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting file comment.";
		return;
	}

	fsfFrameCtr++;
}

int ADIToFRecorder::findDigits(int number)
{
	int numOfDigits = 0;

	do
	{
		numOfDigits++;
		number /= 10;
	} while (number);

	return numOfDigits;
}

void ADIToFRecorder::stopRecording() 
{
	m_recordTreadStop = true;	
	if (m_recordThread.joinable()) 
	{
		m_recordThread.join();
	}
	if (isFSFRecording)
	{
		if (pFsfWrite != NULL)
		{
			LOG(INFO) << "Please wait: Writing XYZ data to FSF file...";
			pFsfWrite->SaveFile();
			pFsfWrite->CloseFile();
			delete pFsfWrite;
			pFsfWrite = NULL;
		}
		//RAW
		if (pFsfWriteRaw != NULL)
		{
			LOG(INFO) << "Please wait: Writing RAW data to FSF file...";
			pFsfWriteRaw->SaveFile();
			pFsfWriteRaw->CloseFile();
			delete pFsfWriteRaw;
			pFsfWriteRaw = NULL;
			fsfFrameCtr = 0;
		}
	}
	else
	{
		m_recordFile.close();
	}
	LOG(INFO) << "Recording has been completed.";
}

int ADIToFRecorder::startPlayback(const std::string& fileName, int& fps) 
{
	if (fileName.find(".raw") != std::string::npos)//if a raw file was found
	{
		return startPlaybackRaw(fileName, fps);
	}
	else if (fileName.find(".fsf") != std::string::npos)//If fsf File was found
	{
		pFsfRead = new aditof::FSF_Common{ aditof::FsfMode::READ, FSF_CUSTOM_MAX_NUM_FRAMES };
		pFsfRead->OpenFile(fileName.c_str());
		startPlaybackFSF();
		return 1;
	}
	else 
	{
		return 0;//No file type is supported
	}
}

bool ADIToFRecorder::startPointCloudBinToFSFConversion(const std::string& fileName, int& frames, int& width, int& height)
{
	ADIBinToFSF* convert = new ADIBinToFSF();	
	convert->ConvertPointCloudToFSF(fileName, frames, width, height);
	return true;
}

int ADIToFRecorder::startPlaybackRaw(const std::string& fileName, int& fps)
{
	unsigned int height = 0;
	unsigned int width = 0;

	m_playbackFile.open(fileName, std::ios::binary);

	m_playbackFile.seekg(0, std::ios_base::end);
	fileSize = m_playbackFile.tellg();
	m_playbackFile.seekg(0, std::ios_base::beg);

	m_playbackFile.read(reinterpret_cast<char*>(&height), sizeof(int));
	m_playbackFile.read(reinterpret_cast<char*>(&width), sizeof(int));
	m_playbackFile.read(reinterpret_cast<char*>(&fps), sizeof(int));
	int sizeOfHeader = 3 * sizeof(int);
	int sizeOfFrame = sizeof(uint16_t) * height * width;
	currentPBPos = sizeOfHeader;
	m_numberOfFrames = (fileSize - sizeOfHeader) / sizeOfFrame;

	m_frameDetails.height = height;
	m_frameDetails.width = width;

	m_playbackThreadStop = false;
	m_playBackEofReached = false;
	LOG(INFO) << "Streaming using raw or bin format. ";
	m_playbackThread =
		std::thread(std::bind(&ADIToFRecorder::playbackThread, this));

	return m_numberOfFrames;
}

void ADIToFRecorder::startPlaybackFSF()
{	
	//Assuming pFsfRead is not null
	if (nullptr != pFsfRead)
	{
		//Clear Dictionary containing Stream index information
		streamTable.clear();
		_streamEnable.active_br = false;
		_streamEnable.radial = false;
		_streamEnable.x = false;
		_streamEnable.y = false;
		_streamEnable.depth = false;
		//Get File Header
		if (pFsfRead->GetFileHeader(fileHeader) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not load FSF file, error on getting file header.";
			return;
		}
		
		for (int idx=0; idx<fileHeader.nStreams; idx++)
		{
			if (pFsfRead->GetStreamInfo(idx, streamInfo) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not load FSF file, error on getting stream info for %d index.", idx;
				return;
			}
			enableStreamType((aditof::StreamType)streamInfo.StreamType);
			streamTable[streamInfo.StreamType] = idx;
		}
		
		//Get Optional file Header
		if (pFsfRead->GetOptionalFileHeader(optFileHeader) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not load FSF file, error on getting optional file header.";
			return;
		}
		//Get File Comment
		if (pFsfRead->GetFileComment(fileComment) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not load FSF file, error on getting file comment.";
			return;
		}

		m_frameDetails.height = streamInfo.nRowsPerStream;
		m_frameDetails.width = streamInfo.nColsPerStream;		

		m_playbackThreadStop = false;
		m_playBackEofReached = false;
		LOG(INFO) << "Streaming using FSF format. ";
		m_playbackThread =
			std::thread(std::bind(&ADIToFRecorder::playbackFSFThread, this));
	}
}

void ADIToFRecorder::enableStreamType(aditof::StreamType streamType)
{
	switch (streamType)
	{
	case aditof::StreamType::STREAM_TYPE_ACTIVE_BR:
		_streamEnable.active_br = true;
		break;
	case aditof::StreamType::STREAM_TYPE_RADIAL:
		_streamEnable.radial = true;
		break;
	case aditof::StreamType::STREAM_TYPE_X:
		_streamEnable.x = true;
		break;
	case aditof::StreamType::STREAM_TYPE_Y:
		_streamEnable.y = true;
		break;
	case aditof::StreamType::STREAM_TYPE_DEPTH:
		_streamEnable.depth = true;
		break;
	default:
		break;
	}
}

void ADIToFRecorder::stopPlayback() 
{
	m_playbackThreadStop = true;
	std::unique_lock<std::mutex> lock(m_playbackMutex);
	m_shouldReadNewFrame = true;
	lock.unlock();
	m_playbackCv.notify_one();
	if (m_playbackThread.joinable()) {
		m_playbackThread.join();
	}
	m_playbackFile.close();
	if (pFsfRead != NULL)
	{
		pFsfRead->CloseFile();
		delete pFsfRead;
		pFsfRead = NULL;		
	}
	if (pFsfReadRaw != NULL)
	{
		pFsfReadRaw->CloseFile();
		delete pFsfReadRaw;
		pFsfReadRaw = NULL;
	}
}

void ADIToFRecorder::recordNewFrame(std::shared_ptr<aditof::Frame> frame) 
{
	m_recordQueue.enqueue(frame);
}

std::shared_ptr<aditof::Frame> ADIToFRecorder::readNewFrame() 
{
	return m_playbackQueue.dequeue();
}

void ADIToFRecorder::requestFrame() 
{
	std::unique_lock<std::mutex> lock(m_playbackMutex);
	m_shouldReadNewFrame = true;
	lock.unlock();
	m_playbackCv.notify_one();
}

bool ADIToFRecorder::isRecordingEnabled() const 
{
	return !m_recordTreadStop;
}

bool ADIToFRecorder::isPlaybackEnabled() const 
{
	return !m_playbackThreadStop;
}

bool ADIToFRecorder::isPlaybackFinished() const 
{
	return m_playBackEofReached;
}

bool ADIToFRecorder::isRecordingFinished() const
{
	return m_finishRecording;
}

bool ADIToFRecorder::isPlaybackPaused()
{
	return isPaused;
}

void ADIToFRecorder::setPlaybackPaused(bool paused)
{
	isPaused = paused;
}

int ADIToFRecorder::getNumberOfFrames() const { return m_numberOfFrames; }

void ADIToFRecorder::recordThread() 
{
	clock_t recording_start = clock();
	while (!m_recordTreadStop) 
	{
		int checkTime = (clock() - recording_start)/ CLOCKS_PER_SEC;
		if (!m_recordFile.is_open()) {
			break;
		}

		if (m_recordQueue.empty()) {
			continue;
		}
		if (frameCtr/*checkTime*/ >= framesToRecord)
		{			
			frameCtr = 0;//reset.
			m_finishRecording = true;
			break;
		}

		auto frame = m_recordQueue.dequeue();

		uint16_t* irData;
		frame->getData("ir", &irData);

		uint16_t* depthData;
		frame->getData("depth", &depthData);

		//Raw Data
		uint16_t* rawData;
		frame->getData( "raw", &rawData);

		int width = m_frameDetails.width;
		int height = m_frameDetails.height;
		int captures = m_frameDetails.totalCaptures;
		int size = static_cast<int>(sizeof(uint16_t) * width * height);
		
		m_recordFile.write(reinterpret_cast<const char*>(irData), size);

		m_recordFile.write(reinterpret_cast<const char*>(depthData), size);

		//Create a new .bin file for each frame with raw sensor data
		std::string fileNameRawFrame = m_fileNameRaw;
		fileNameRawFrame.replace(fileNameRawFrame.find_last_of("#"), 1, std::to_string(frameCtr));
		std::ofstream recordFileRaw;
		recordFileRaw.open(fileNameRawFrame, std::ios::binary | std::ofstream::trunc);
		recordFileRaw.write(reinterpret_cast<const char*>(rawData), static_cast<int>(captures * size));
		recordFileRaw.close();

		frameCtr++;
	}
}

void ADIToFRecorder::recordFSFThread()
{
	while (!m_recordTreadStop)
	{
		if (m_recordQueue.empty())
		{
			continue;
		}

		if (frameCtr >= framesToRecord || fsfStop)
		{
			recordFSFThreadFail();
			break;
		}

		auto frame = m_recordQueue.dequeue();		

		aditof::FrameDetails frameDetails;
		frame->getDetails(frameDetails);
		size_t size = frameDetails.height * frameDetails.width * sizeof(uint16_t);

		//Get the frames
		if (_streamEnable.active_br)
		{
			//Create Stream Info IR, sstreamIdx = 0
			uint16_t* irData;
			frame->getData("ir", &irData);
			stream.streamHeader.TimeStamp = 0;//Put real timestamp
			stream.streamHeader.CompressedStreamSize = 0; //put real value
			stream.optionalStreamHeader = "ADI"; //put real value
			stream.streamComment = "IR";//put real value			
			stream.streamData.assign(reinterpret_cast<const char*>(irData), reinterpret_cast<const char*>(irData) + size);
			if (pFsfWrite->SetStream(frameCtr, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR)->second, stream) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not set stream.";
				recordFSFThreadFail();
				break;				
			}
		}

		if (_streamEnable.radial)
		{
			//Create Stream Info DEPTH, sstreamIdx = 1	
			uint16_t* depthData;
			frame->getData("depth", &depthData);
			stream.streamHeader.TimeStamp = 0;//Put real timestamp
			stream.streamHeader.CompressedStreamSize = 0; //put real value
			stream.optionalStreamHeader = "ADI"; //put real value
			stream.streamComment = "DEPTH";//put real value		
			stream.streamData.assign(reinterpret_cast<const char*>(depthData), reinterpret_cast<const char*>(depthData) + size);
			if (pFsfWrite->SetStream(frameCtr, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_RADIAL)->second, stream) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not set stream.";
				recordFSFThreadFail();
				break;
			}

			if (_streamEnable.depth && _streamEnable.x && _streamEnable.y)
			{
				//Create Stream Info XYZ, sstreamIdx = 2, 3, and 4
				uint16_t* xyzData;//The whole Point Cloud buffer, however, we need to break it out into x and y for backwards compatibility
				uint16_t* xData = new uint16_t[frameDetails.height * frameDetails.width * sizeof(uint16_t)];
				uint16_t* yData = new uint16_t[frameDetails.height * frameDetails.width * sizeof(uint16_t)];
				uint16_t* zData = new uint16_t[frameDetails.height * frameDetails.width * sizeof(uint16_t)];
				size_t pcsize = frameDetails.height * frameDetails.width * sizeof(aditof::Point3I);
				size_t xBuffCnt = 0;
				size_t yBuffCnt = 0;
				size_t zBuffCnt = 0;
				frame->getData("xyz", &xyzData);

				stream.streamHeader.TimeStamp = frameCtr;
				stream.optionalStreamHeader = "ADI"; //put real value
				stream.streamComment = "X";//put real value
				
				for (size_t xyzbuffCnt = 0; xyzbuffCnt < pcsize; xyzbuffCnt++)
				{
					//xyzData = [XX, YY, ZZ]...
					//Get only X component
					xData[xBuffCnt++] = xyzData[xyzbuffCnt++];
					//Get only Y component
					yData[yBuffCnt++] = xyzData[xyzbuffCnt++];
					//Get only Z component
					zData[zBuffCnt++] = xyzData[xyzbuffCnt];
				}
				stream.streamHeader.CompressedStreamSize = xBuffCnt;

				//X component
				stream.streamData.assign(reinterpret_cast<const char*>(xData), reinterpret_cast<const char*>(xData) + xBuffCnt);
				int xIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_X)->second;
				if (pFsfWrite->SetStream(frameCtr, xIndex, stream) != aditof::FsfStatus::SUCCESS)
				{
					LOG(ERROR) << "Could not set stream.";
					recordFSFThreadFail();
					delete[] xData;
					delete[] yData;
					delete[] zData;
					break;
				}

				//Y component
				stream.streamHeader.TimeStamp = frameCtr;
				stream.streamHeader.CompressedStreamSize = yBuffCnt;
				stream.optionalStreamHeader = "ADI"; //put real value
				stream.streamComment = "Y";//put real value
				stream.streamData.assign(reinterpret_cast<const char*>(yData), reinterpret_cast<const char*>(yData) + yBuffCnt);
				int yIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_Y)->second;
				if (pFsfWrite->SetStream(frameCtr, yIndex, stream) != aditof::FsfStatus::SUCCESS)
				{
					LOG(ERROR) << "Could not set stream.";
					recordFSFThreadFail();
					delete[] xData;
					delete[] yData;
					delete[] zData;
					break;
				}

				//Z component
				stream.streamHeader.TimeStamp = frameCtr;
				stream.streamHeader.CompressedStreamSize = zBuffCnt;
				stream.optionalStreamHeader = "ADI"; //put real value
				stream.streamComment = "Z";//put real value
				stream.streamData.assign(reinterpret_cast<const char*>(zData), reinterpret_cast<const char*>(zData) + zBuffCnt);
				int zIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH)->second;
				if (pFsfWrite->SetStream(frameCtr, zIndex, stream) != aditof::FsfStatus::SUCCESS)
				{
					LOG(ERROR) << "Could not set stream.";
					recordFSFThreadFail();
					delete[] xData;
					delete[] yData;
					delete[] zData;
					break;
				}

				delete[] xData;
				delete[] yData;
				delete[] zData;
			}		
		}			
		
		//Get the RAW frames
		//Create Stream Info RAW, streamIdx = 0
		uint16_t* rawData;
		frame->getData("raw", &rawData);

        uint16_t *pHeader = nullptr;
        frame->getData("embedded_header", &pHeader);  

		for (size_t streamCnt = 0; streamCnt < fileHeaderRaw.nStreams; streamCnt++)
		{
			streamRaw.streamHeader.TimeStamp = frameCtr;
			streamRaw.streamHeader.CompressedStreamSize = size;
			streamRaw.optionalStreamHeader.assign(reinterpret_cast<const char*>(pHeader) + (EMBED_HDR_LENGTH * streamCnt), EMBED_HDR_LENGTH);
			streamRaw.streamComment = "RAW";
			streamRaw.streamData.assign(reinterpret_cast<const char*>(rawData) + (size * streamCnt), reinterpret_cast<const char*>(rawData) + size + (size * streamCnt));

			if (pFsfWriteRaw->SetStream(frameCtr, streamCnt, streamRaw) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not set stream.";
				recordFSFThreadFail();
				break;
			}
		}

		frameCtr++;
	}
}

void ADIToFRecorder::recordFSFThreadFail()
{
	frameCtr = 0;//reset.
	m_finishRecording = true;
	fsfStop = false;
}
void ADIToFRecorder::playbackThread() 
{		
	int sizeOfFrame = 0;
	while (!m_playbackThreadStop) 
	{

		if (!m_playbackFile.is_open()) 
		{
			break;
		}

		std::unique_lock<std::mutex> lock(m_playbackMutex);
		m_playbackCv.wait(lock, [&]() { return m_shouldReadNewFrame; });
		m_shouldReadNewFrame = false;

		if (m_playbackThreadStop) 
		{
			break;
		}

		auto frame = std::make_shared<aditof::Frame>();		
		frame->setDetails(m_frameDetails);
		
		frame->getData("ir", &frameDataLocationIR);
		frame->getData("depth", &frameDataLocationDEPTH);

		unsigned int width = m_frameDetails.width;
		unsigned int height = m_frameDetails.height;
		
		sizeOfFrame = sizeof(uint16_t) * height * width;

		if (m_playbackFile.eof()) 
		{
			memset(frameDataLocationIR, 0, sizeof(uint16_t) * width * height);
			m_playBackEofReached = true;
		}
		else 
		{
			if (!isPaused && (currentPBPos < (fileSize - (sizeOfFrame)*4)))
			{
				int size = static_cast<int>(sizeof(uint16_t) * width * height);
				m_playbackFile.seekg(currentPBPos);
				currentPBPos += size * 2;
				m_playbackFile.read(reinterpret_cast<char*>(frameDataLocationIR),
					size);
				m_playbackFile.read(reinterpret_cast<char*>(frameDataLocationDEPTH),
					size);
				
			}
			else
			{
				m_playbackFile.seekg(currentPBPos);
				int size = static_cast<int>(sizeof(uint16_t) * width * height);
				m_playbackFile.read(reinterpret_cast<char*>(frameDataLocationIR),
					size);
				m_playbackFile.read(reinterpret_cast<char*>(frameDataLocationDEPTH),
					size);	
			}					
		}
		
		m_playbackQueue.enqueue(frame);
	}
}

void ADIToFRecorder::clearVariables()
{
	frameDataLocationIR = nullptr;
	frameDataLocationDEPTH = nullptr;
	frameDataLocationXYZ = nullptr;
	stream.streamData.clear();
	streamRaw.streamData.clear();
	m_frameDetails.height = 0;
	m_frameDetails.passiveIRCaptured = false;
	m_frameDetails.cameraMode = "";
	m_frameDetails.totalCaptures = 0;
	m_frameDetails.type = "";
	m_frameDetails.width = 0;
	streamTable.clear();
}

void ADIToFRecorder::playbackFSFThread()
{
	currentPBPos = 0;//Current Playback position
	while (!m_playbackThreadStop)
	{
		if (m_frameDetails.height == 0 || m_frameDetails.height >= 2048)
		{
			LOG(ERROR) << "Failed to load data file.";
			clearVariables();
			break;
		}

		std::unique_lock<std::mutex> lock(m_playbackMutex);
		m_playbackCv.wait(lock, [&]() { return m_shouldReadNewFrame; });
		m_shouldReadNewFrame = false;

		auto frame = std::make_shared<aditof::Frame>();

		if (m_playbackThreadStop)
		{
			clearVariables();
			break;
		}
		//Previous frameDetails values. We need this to avoid size mismatch
		unsigned int _prevWidth = m_frameDetails.width;
		unsigned int _prevHeight = m_frameDetails.height;

		m_frameDetails.width = streamInfo.nColsPerStream;
		m_frameDetails.height = streamInfo.nRowsPerStream;
		m_frameDetails.type = "";
		m_frameDetails.totalCaptures = 1;

		const std::vector<std::string> recordedTypes = {"ir", "depth", "xyz"};
		for (unsigned int i = 0; i < recordedTypes.size(); i++) {
			aditof::FrameDataDetails fDetails;
			fDetails.type = recordedTypes[i];
			fDetails.width = streamInfo.nColsPerStream;
			fDetails.height = streamInfo.nRowsPerStream;			
			m_frameDetails.dataDetails.emplace_back(fDetails);
		}

		frame->setDetails(m_frameDetails);
		frame->getData("ir", &frameDataLocationIR);
		frame->getData("depth", &frameDataLocationDEPTH);
		frame->getData("xyz", &frameDataLocationXYZ);

		//Verify that the buffers are loaded and match the required size
		if ((frameDataLocationIR != nullptr && frameDataLocationDEPTH != nullptr)
			&& (_prevWidth == streamInfo.nColsPerStream && _prevHeight == streamInfo.nRowsPerStream))
		{
			
			if (currentPBPos  == (fileHeader.nFrames-1))
			{
				m_playBackEofReached = true;
			}
			else 
			{
				m_playBackEofReached = false;
			}

			if ((currentPBPos < (fileHeader.nFrames-1)) && !isPaused)
			{
				//Get IR Stream Info
				if (_streamEnable.active_br)
				{
					pFsfRead->GetStream(currentPBPos, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR)->second, stream);
					memcpy(reinterpret_cast<char*>(frameDataLocationIR), stream.streamData.data(), stream.streamData.size());
					stream.streamData.clear();
				}
				//Get DEPTH Stream Info
				if (_streamEnable.radial)
				{
					pFsfRead->GetStream(currentPBPos, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_RADIAL)->second, stream);
					memcpy(reinterpret_cast<char*>(frameDataLocationDEPTH), stream.streamData.data(), stream.streamData.size());
					stream.streamData.clear();
				}
				if (_streamEnable.x && _streamEnable.y)//This must be a Point Cloud
				{
					processXYZData();
				}
				currentPBPos++;
			}
			else //loop around, and keep last stream showing
			{
				stream.streamData.clear();
				stream.optionalStreamHeader.clear();
				stream.streamComment.clear();
				//Get IR Stream Info
				if (_streamEnable.active_br)
				{
					pFsfRead->GetStream(currentPBPos, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR)->second, stream);
					memcpy(reinterpret_cast<char*>(frameDataLocationIR), stream.streamData.data(), stream.streamData.size());
				}
				//Get DEPTH Stream Info
				if (_streamEnable.radial)
				{
					pFsfRead->GetStream(currentPBPos, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_RADIAL)->second, stream);
					memcpy(reinterpret_cast<char*>(frameDataLocationDEPTH), stream.streamData.data(), stream.streamData.size());
					//if (_streamEnable.x && _streamEnable.y)//Get Point Cloud Information
					//{
					//	processXYZData();
					//}
				}
				if (_streamEnable.x && _streamEnable.y)//Get Point Cloud Information
				{
					processXYZData();
				}
			}
		}

		m_playbackQueue.enqueue(frame);
	}
}

void ADIToFRecorder::processXYZData()
{
	size_t imageSize = m_frameDetails.width * m_frameDetails.height * sizeof(uint16_t);
	//local variables
	uint16_t* xData = new uint16_t[imageSize];
	uint16_t* yData = new uint16_t[imageSize];
	uint16_t* zData = new uint16_t[imageSize];
	
	
	//Get X
	int xIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_X)->second;
	pFsfRead->GetStream(currentPBPos, xIndex, stream);
	memcpy(reinterpret_cast<char*>(xData), stream.streamData.data(), stream.streamData.size());
	stream.streamData.clear();
	
	//Get Y
	int yIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_Y)->second;
	pFsfRead->GetStream(currentPBPos, yIndex, stream);
	memcpy(reinterpret_cast<char*>(yData), stream.streamData.data(), stream.streamData.size());
	stream.streamData.clear();
	
	//Get Z
	int zIndex = streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH)->second;
	pFsfRead->GetStream(currentPBPos, zIndex, stream);
	memcpy(reinterpret_cast<char*>(zData), stream.streamData.data(), stream.streamData.size());
	stream.streamData.clear();

	//Now that we have X, Y, and Z streams, we need to put all data together to form XYZ stream
	size_t pcSize = m_frameDetails.width * m_frameDetails.height * sizeof(aditof::Point3I);
	size_t depthCnt = 0;
	size_t xCnt = 0;
	size_t yCnt = 0;
	size_t zCnt = 0;
	for (size_t xyzBuffCnt = 0; xyzBuffCnt < pcSize; xyzBuffCnt++)
	{
		//Get X
		frameDataLocationXYZ[xyzBuffCnt++] = xData[xCnt++];
		//Get Y
		frameDataLocationXYZ[xyzBuffCnt++] = yData[yCnt++];
		//Get Z
		frameDataLocationXYZ[xyzBuffCnt] = zData[zCnt++];
	}
	delete[] xData;
	delete[] yData;
	delete[] zData;
}
