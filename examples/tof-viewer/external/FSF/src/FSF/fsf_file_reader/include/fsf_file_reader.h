// Copyright (C) Microsoft Corporation. All rights reserved.

#pragma once

#include "fsf_wrapper.h"

namespace FileUtils
{
	class FSFFileReader
	{
	public:
		static const int nMaxStreams = 10;

		FSFFileReader(char* filename);
		unsigned int GetFrameCount();
		bool GetStreamInfo(unsigned int streamID, FSFSTREAMINFO& streamInfo);
		void* GetFramePtr(unsigned int frameID);
		void* GetFramePtr(unsigned int frameID, char* optStreamHdr);
		bool GetFileHeader(FSFFILEHEADER& fileHeader);
		bool GetOptionalHeader(void* m_OptionalFileHeader);
		bool IsXYZFile();
		bool IsValid() { return m_IsHandleValid; }
		~FSFFileReader();

	private:
		unsigned int m_FileHandle;
		bool m_IsHandleValid;
		FSFFILEHEADER m_FileHeader;
		FSFSTREAMINFO m_StreamInfo[nMaxStreams];
		unsigned char* m_FrameBuffer;
	};
}

