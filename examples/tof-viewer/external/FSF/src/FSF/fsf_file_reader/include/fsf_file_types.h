// Copyright (C) Microsoft Corporation. All rights reserved.

#pragma once
#ifndef _FSF_FILE_TYPES_
#define _FSF_FILE_TYPES_

//Constants
const unsigned int   FSF_FILE_MAGIC_NUMBER = 0x00465346; //On little endian machines this should write FSF
const unsigned short FILE_FORMAT_MAJOR_VER = 1;
const unsigned short FILE_FORMAT_MINOR_VER = 1;

//Type definitions
enum FILE_CREATION_FLAGS   {FILE_CREATION_FLAGS_CREATE = 0, FILE_CREATION_FLAGS_CREATE_IF_NOT_EXISTS};

enum FILE_MODE             {FILE_MODE_READ = 0, FILE_MODE_WRITE, FILE_MODE_NOT_SET};

enum FSF_HDR_FIELD_OFFSETS {FSF_FIELD_HeaderSize = 0, FSF_FIELD_MagicNumber = 4, FSF_FIELD_FileFormatVersion = 8, FSF_FIELD_FileHdrSize = 12, FSF_FIELD_StreamInfoSize = 16, 
							FSF_FIELD_StreamHeaderSize = 20, FSF_FIELD_OptionalFileHdrSize = 24, FSF_FIELD_FileCommentsSize = 28, FSF_FIELD_FrameOffsetInfoLoc = 32,
							FSF_FIELD_nFrames = 40, FSF_FIELD_nStreams = 44}; 

enum FSF_CHANNEL_FORMAT    {FSF_CHANNEL_UNSIGNED_BYTE = 1, FSF_CHANNEL_SIGNED_BYTE, FSF_CHANNEL_UINT16, FSF_CHANNEL_SIGNED_INT16, FSF_CHANNEL_UINT32, FSF_CHANNEL_INT32, 
							FSF_CHANNEL_UINT64, FSF_CHANNEL_INT64, FSF_CHANNEL_FLOAT, FSF_CHANNEL_DOUBLE};

//enum SYSTEM_ID             {SYSTEM_ID_UNKNOWN = 0, SYSTEM_ID_COBRA_PIXART, SYSTEM_ID_COBRA_BROADCOM, SYSTEM_ID_HUGO_BROADCOM};

enum STREAM_TYPE           {STREAM_TYPE_UNKNOWN = 0, STREAM_TYPE_RAW, STREAM_TYPE_ACTIVE_BR, STREAM_TYPE_PHASE, STREAM_TYPE_X, STREAM_TYPE_Y, STREAM_TYPE_DEPTH, STREAM_TYPE_RGB, STREAM_TYPE_RAW_NORM, 
							STREAM_TYPE_RAW_REAL, STREAM_TYPE_RAW_IMAG, STREAM_TYPE_RAW_REAL_FILT, STREAM_TYPE_RAW_IMAG_FILT, STREAM_TYPE_RADIAL, STREAM_TYPE_RADIAL_FILT, STREAM_TYPE_COMMON_MODE, STREAM_TYPE_CONF,
                            STREAM_TYPE_VARIANCE, STREAM_TYPE_REFLECTIVITY };

enum COMPRESSION_TYPE      {COMPRESSION_TYPE_NONE  = 0};


struct FSFFILEHEADER
{
	unsigned int HeaderSize;        //This is size of FSFFILEHEADER + Any number of FSFSTREAMINFO blocks
	unsigned int MagicNumber;	
	unsigned int FileFormatVersion;

	unsigned int FileHeaderSize;
	unsigned int StreamInfoSize;
	unsigned int StreamHeaderSize;

	unsigned int OptionalFileHdrSize;
	unsigned int FileCommentSize;
	long long FrameOffsetInfoLoc; //Offset in bytes from beginning of the file
	unsigned int nFrames;
	unsigned int nStreams;
};

struct FSFSTREAMINFO
{
	unsigned int SystemID;        //Processor and Sensor Combination
	unsigned int StreamType;      //Types of streams e.g. Active Brightness, Phase, RAW, X, Y, Z
	unsigned int ChannelFormat;
	unsigned int BytesPerPixel;
	unsigned int nRowsPerStream;
	unsigned int nColsPerStream;
	unsigned int OptionalStreamHdrSize;
	unsigned int StreamCommentSize;
	unsigned int CompressionScheme;
};

struct FSFSTREAMHEADER
{
	unsigned int TimeStamp;
	unsigned int CompressedStreamSize;  //When no compression is applied this would be the actual stream size
};

#endif