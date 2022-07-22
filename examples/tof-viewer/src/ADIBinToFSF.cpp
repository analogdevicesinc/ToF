/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "ADIBinToFSF.h"

ADIBinToFSF::ADIBinToFSF()
{

}

ADIBinToFSF::~ADIBinToFSF()
{}

aditof::FSF* ADIBinToFSF::ConvertPointCloudToFSF(const std::string& fileName, int& frames, int& width, int& height)
{
	aditof::FSF* fsfWrite;
	std::ifstream readFile;
	readFile.open(fileName, std::ios::binary);	
	size_t fileSize = sizeof(aditof::Point3I) * width * height;
	uint16_t* frameDataLocationXYZ = new uint16_t[fileSize];
	if (readFile.is_open())
	{
		readFile.seekg(0);;//Start from the beginning. It does not have any header (for now)
		readFile.read(reinterpret_cast<char*>(frameDataLocationXYZ),
			fileSize);
		readFile.close();
	}
	fsfWrite = new aditof::FSF_Common{ aditof::FsfMode::WRITE, (const unsigned int)frames };
	if (fsfWrite->CreateFsfFile(fileName.c_str()) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file";
		return nullptr;
	}
	aditof::FileHeader fileHeader;
	FSFStreamEnable _streamEnable;
	aditof::StreamInfo streamInfo;
	aditof::OptionalFileHeader optFileHeader;
	aditof::FileComment fileComment;
	aditof::Stream stream;
	_streamEnable.active_br = false;
	_streamEnable.depth = true;
	_streamEnable.x = true;
	_streamEnable.y = true;
	//Create File Header
	fileHeader.HeaderSize = 120;//372 ;//Verify this
	fileHeader.MagicNumber = FSF_MAGIC_NUMBER;
	fileHeader.FileFormatMajorVersion = FILE_FORMAT_MAJOR_VERSION;
	fileHeader.FileFormatMinorVersion = FILE_FORMAT_MINOR_VERSION;
	fileHeader.FileHeaderSize = 48; //Verify this
	fileHeader.StreamInfoSize = 36; //Verify this
	fileHeader.StreamHeaderSize = 8;
	fileHeader.OptionalFileHdrSize = 3;//211 ;
	fileHeader.FileCommentSize = 0;
	fileHeader.FrameOffsetInfoLoc = 100;//167773443;//16774423;//167773651;//Verify this
	fileHeader.nFrames = frames;// fps;
	int _numStreams = 0;
	if (_streamEnable.active_br)
		_numStreams++;
	if (_streamEnable.depth)
		_numStreams++;
	if (_streamEnable.x)
		_numStreams++;
	if (_streamEnable.y)
		_numStreams++;
	fileHeader.nStreams = _numStreams;

	if (fsfWrite->SetFileHeader(fileHeader) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error while creating file header.";
		return nullptr;
	}

	int _streamCounter = 0;
	//Clear Dictionary containing Stream index information
	//streamTable.clear();
	
	if (_streamEnable.active_br)
	{
		//Stream Information IR	
		//Create Stream Info IR, streamIdx = 0
		streamInfo.SystemID = 0;//Verify this concept
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR;
		streamInfo.ChannelFormat = 0;//Verify this concept
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;//3;//2048 ;//Hardcoded, verify this concept
		streamInfo.StreamCommentSize = 2;//Verify this concept
		streamInfo.CompressionScheme = 0;//verify this concept		
		if (fsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for ACTIVE_BR.";
			return nullptr;
		}
		//streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR] = _streamCounter;
		_streamCounter++;
	}

	if (_streamEnable.depth)//Only Depth
	{
		//Stream Information DEPTH
		//Create Stream Info DEPTH, streamIdx = 1
		streamInfo.SystemID = 15;//Verify this concept
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH;
		streamInfo.ChannelFormat = 4;//Verify this concept
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;//3;//2048 ;//Hardcoded, verify this concept
		streamInfo.StreamCommentSize = 5;//Verify this concept
		streamInfo.CompressionScheme = 0;//verify this concept
		if (fsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for DEPTH.";
			return nullptr;
		}
		//streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH] = _streamCounter;
		_streamCounter++;
	}

	if (_streamEnable.x)
	{
		//Stream Information X
		//Create Stream Info X, streamIdx = 2
		streamInfo.SystemID = 16;//Verify this concept
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_X;
		streamInfo.ChannelFormat = 4;//Verify this concept
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;//3;//2048 ;//Hardcoded, verify this concept
		streamInfo.StreamCommentSize = 1;//Verify this concept
		streamInfo.CompressionScheme = 0;//verify this concept
		if (fsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_X.";
			return nullptr;
		}
		//streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_X] = _streamCounter;
		_streamCounter++;
	}
	if (_streamEnable.y)
	{
		//Stream Information Y
		//Create Stream Info Y, streamIdx = 3
		streamInfo.SystemID = 17;//Verify this concept
		streamInfo.StreamType = (uint32_t)aditof::StreamType::STREAM_TYPE_Y;
		streamInfo.ChannelFormat = 4;//Verify this concept
		streamInfo.BytesPerPixel = 2;
		streamInfo.nRowsPerStream = height;
		streamInfo.nColsPerStream = width;
		streamInfo.OptionalStreamHdrSize = 3;//3;//2048 ;//Hardcoded, verify this concept
		streamInfo.StreamCommentSize = 1;//Verify this concept
		streamInfo.CompressionScheme = 0;//verify this concept
		if (fsfWrite->SetStreamInfo(_streamCounter, streamInfo) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not create FSF file, error while creating stream info for TYPE_Y.";
			return nullptr;
		}
		//streamTable[(uint32_t)aditof::StreamType::STREAM_TYPE_Y] = _streamCounter;
		_streamCounter++;
	}

	//Create Optional File header	
	optFileHeader = "ADI";//TODO: Make a real optional file header
	stream.optionalStreamHeader = optFileHeader;
	if (fsfWrite->SetOptionalFileHeader(optFileHeader) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting optional header file.";
		return nullptr;
	}

	//File Comment
	fileComment = "";
	if (fsfWrite->SetFileComment(fileComment) != aditof::FsfStatus::SUCCESS)
	{
		LOG(ERROR) << "Could not create FSF file, error on setting file comment.";
		return nullptr;
	}

	//Create the FSF File Below:
	//Get the frames
	size_t imageSize =width * height * sizeof(uint16_t);
	//local variables
	uint16_t* xData = new uint16_t[imageSize];
	uint16_t* yData = new uint16_t[imageSize];
	uint16_t* zData = new uint16_t[imageSize];//Depth
	//frameDataLocationXYZ
	size_t xCnt = 0;
	size_t yCnt = 0;
	size_t zCnt = 0;
	for (size_t xyzBuffCnt = 0; xyzBuffCnt < fileSize; xyzBuffCnt++)
	{
		//Get X
		xData[xCnt++] = frameDataLocationXYZ[xyzBuffCnt++];
		//Get Y
	    yData[yCnt++] = frameDataLocationXYZ[xyzBuffCnt++];
		//Get Z or Depth
		zData[zCnt++] = frameDataLocationXYZ[xyzBuffCnt];
	}

	//if (_streamEnable.active_br)
	//{
	//	//Create Stream Info IR, sstreamIdx = 0
	//	uint16_t* irData;
	//	frame->getData(aditof::FrameDataType::IR, &irData);
	//	stream.streamHeader.TimeStamp = 0;//Put real timestamp
	//	stream.streamHeader.CompressedStreamSize = 0; //put real value
	//	stream.optionalStreamHeader = "ADI"; //put real value
	//	stream.streamComment = "IR";//put real value			
	//	stream.streamData.assign(reinterpret_cast<const char*>(irData), reinterpret_cast<const char*>(irData) + size);
	//	if (pFsfWrite->SetStream(frameCtr, streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_ACTIVE_BR)->second, stream) != aditof::FsfStatus::SUCCESS)
	//	{
	//		LOG(ERROR) << "Could not set stream.";
	//		recordFSFThreadFail();
	//		break;
	//	}
	//	if (pFsfWrite->SaveFile() != aditof::FsfStatus::SUCCESS)
	//	{
	//		LOG(ERROR) << "Error while saving the ACTIVE_BR stream.";
	//		recordFSFThreadFail();
	//		break;
	//	}
	//}

	if (_streamEnable.depth)
	{
		//Create Stream Info DEPTH, sstreamIdx = 1	
		/*uint16_t* depthData;
		frame->getData(aditof::FrameDataType::DEPTH, &depthData);*/
		stream.streamHeader.TimeStamp = 0;//Put real timestamp
		stream.streamHeader.CompressedStreamSize = 0; //put real value
		stream.optionalStreamHeader = "ADI"; //put real value
		stream.streamComment = "DEPTH";//put real value		
		stream.streamData.assign(reinterpret_cast<const char*>(zData), reinterpret_cast<const char*>(zData) + zCnt);
		if (fsfWrite->SetStream(/*frameCtr*/0, /*streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_DEPTH)->second*/0, stream) != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Could not set stream.";
		}
		if (fsfWrite->SaveFile() != aditof::FsfStatus::SUCCESS)
		{
			LOG(ERROR) << "Error while saving the DEPTH stream.";
		}

		if (_streamEnable.depth && _streamEnable.x && _streamEnable.y)
		{
			//Create Stream Info XYZ, sstreamIdx = 2 and 3
			//uint16_t* xyzData;//The whole Point Cloud buffer, however, we need to break it out into x and y for backwards compatibility
			//uint16_t* xData = new uint16_t[frameDetails.height * frameDetails.width * sizeof(uint16_t)];
			//uint16_t* yData = new uint16_t[frameDetails.height * frameDetails.width * sizeof(uint16_t)];
			//size_t pcsize = frameDetails.height * frameDetails.width * sizeof(aditof::Point3I);
			//size_t xBuffCnt = 0;
			//size_t yBuffCnt = 0;
			//frame->getData(aditof::FrameDataType::XYZ, &xyzData);
			stream.streamHeader.TimeStamp = 0;//Put real timestamp
			stream.streamHeader.CompressedStreamSize = 0; //put real value
			stream.optionalStreamHeader = "ADI"; //put real value
			stream.streamComment = "X";//put real value

			//for (size_t xyzbuffCnt = 0; xyzbuffCnt < pcsize; xyzbuffCnt++)
			//{
			//	//xyzData = [XX, YY, ZZ]...
			//	//Get only X component
			//	xData[xBuffCnt++] = xyzData[xyzbuffCnt++];
			//	//Get only Y component
			//	yData[yBuffCnt++] = xyzData[xyzbuffCnt++];
			//}
			//X component
			stream.streamData.assign(reinterpret_cast<const char*>(xData), reinterpret_cast<const char*>(xData) + xCnt);
			int xIndex = 1;//streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_X)->second;
			if (fsfWrite->SetStream(/*frameCtr*/0, xIndex, stream) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not set stream.";
				delete xData;
				delete yData;
			}
			if (fsfWrite->SaveFile() != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Error while saving the TYPE_X stream.";
				delete xData;
				delete yData;
			}
			//Y component
			stream.streamHeader.TimeStamp = 0;//Put real timestamp
			stream.streamHeader.CompressedStreamSize = 0; //put real value
			stream.optionalStreamHeader = "ADI"; //put real value
			stream.streamComment = "Y";//put real value
			stream.streamData.assign(reinterpret_cast<const char*>(yData), reinterpret_cast<const char*>(yData) + yCnt);
			int yIndex = 2;//streamTable.find((uint32_t)aditof::StreamType::STREAM_TYPE_Y)->second;
			if (fsfWrite->SetStream(/*frameCtr*/0, yIndex, stream) != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Could not set stream.";
				delete xData;
				delete yData;
			}
			if (fsfWrite->SaveFile() != aditof::FsfStatus::SUCCESS)
			{
				LOG(ERROR) << "Error while saving the TYPE_Y stream.";
				delete xData;
				delete yData;
			}

			delete xData;
			delete yData;
		}
	}
	fsfWrite->SaveFile();
	fsfWrite->CloseFile();
	return fsfWrite;
}