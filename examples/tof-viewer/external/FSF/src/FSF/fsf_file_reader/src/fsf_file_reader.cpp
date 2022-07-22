// Copyright (C) Microsoft Corporation. All rights reserved.

#include "fsf_file_reader.h"

using namespace FileUtils;

FSFFileReader::FSFFileReader(char* filename) 
							: m_IsHandleValid(false), m_FrameBuffer(nullptr), m_FileHeader({ 0 })
{
	if (ReadFSFFile(filename, &m_FileHandle) == true)
	{
		m_IsHandleValid = true;
		
		if (ReadFileHeader(m_FileHandle, &m_FileHeader) == true)
		{
			unsigned int frameSize = 0;

			for (unsigned int streamId = 0; streamId < m_FileHeader.nStreams && streamId < nMaxStreams; streamId++)
			{
				if (ReadStreamInfo(m_FileHandle, streamId, m_StreamInfo + streamId) == true)
					frameSize += (m_StreamInfo[streamId].BytesPerPixel * m_StreamInfo[streamId].nColsPerStream * m_StreamInfo[streamId].nRowsPerStream);
			}	

			if (frameSize > 0)
				m_FrameBuffer = new unsigned char[frameSize];
		}	
	}		
	else	
		m_IsHandleValid = false;	
}

unsigned int FSFFileReader::GetFrameCount()
{
	if (m_IsHandleValid)
		return m_FileHeader.nFrames;
	else
		return 0;
}

bool FSFFileReader::GetStreamInfo(unsigned int streamID, FSFSTREAMINFO& streamInfo)
{
	if (m_IsHandleValid && streamID < m_FileHeader.nStreams)
	{
		streamInfo = m_StreamInfo[streamID];
		return true;
	}
	else
		return false;		
}

bool FSFFileReader::GetFileHeader(FSFFILEHEADER& fileHeader)
{
    if (m_IsHandleValid)
    {
        fileHeader = m_FileHeader;
        return true;
    }
    else
        return false;
}

bool FSFFileReader::GetOptionalHeader(void* m_OptionalFileHeader) {
  if (m_IsHandleValid) {
    ReadOptionalFileHdr(m_FileHandle, m_OptionalFileHeader);
    return true;
  } else
    return false;
}

bool FSFFileReader::IsXYZFile()
{
	if (m_IsHandleValid && m_FileHeader.nStreams >= 3 && m_StreamInfo[0].StreamType == STREAM_TYPE_X &&
		m_StreamInfo[1].StreamType == STREAM_TYPE_Y && m_StreamInfo[2].StreamType == STREAM_TYPE_DEPTH)
		return true;
	else
		return false;
}

void* FSFFileReader::GetFramePtr(unsigned int frameID) {
  return this->GetFramePtr(frameID, nullptr);
}

void* FSFFileReader::GetFramePtr(unsigned int frameID, char* optStreamHdr)
{
	if (m_IsHandleValid && frameID < m_FileHeader.nFrames)
	{
		unsigned int streamPtr = 0;
		unsigned int timestamp = 0;
		unsigned int streamHdrPtr = 0;

		for (unsigned int stream = 0; stream < m_FileHeader.nStreams; stream++)
		{
			if(optStreamHdr!=nullptr)
			{
				ReadStream(m_FileHandle, frameID, stream, &timestamp,
                 optStreamHdr + streamHdrPtr, nullptr,
                 m_FrameBuffer + streamPtr);
				streamHdrPtr += m_StreamInfo[stream].OptionalStreamHdrSize;
			}
			else
			{
				ReadStream(m_FileHandle, frameID, stream, &timestamp,
                 nullptr, nullptr, m_FrameBuffer + streamPtr);
			}
			streamPtr += (m_StreamInfo[stream].BytesPerPixel * m_StreamInfo[stream].nColsPerStream * m_StreamInfo[stream].nRowsPerStream);     
		}

		return m_FrameBuffer;
	}
	else
		return nullptr;
}

FSFFileReader::~FSFFileReader()
{
	if (m_IsHandleValid)
	{
		Close(m_FileHandle);
		m_FileHandle = 0;
		m_IsHandleValid = false;
	}

	delete[] m_FrameBuffer;
}










