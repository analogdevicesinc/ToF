/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef FSF_WRAPPER_H
#define FSF_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MAX_FILES   16

typedef enum {
    SUCCESS = 0UL,
    FILE_NOT_OPEN,
    FILE_NOT_CREATED,
    FILE_DOESNOT_EXIST,
    FILE_FORMAT_ERROR,
    FILE_HEADER_ERROR,
    INVALID_OPERATION,
    FAILED
} FSF_ERROR_CODE;

typedef enum {
    FSF_CHANNEL_UNSIGNED_BYTE = 1,
    FSF_CHANNEL_SIGNED_BYTE,
    FSF_CHANNEL_UINT16,
    FSF_CHANNEL_SIGNED_INT16,
    FSF_CHANNEL_UINT32,
    FSF_CHANNEL_INT32,
    FSF_CHANNEL_UINT64,
    FSF_CHANNEL_INT64,
    FSF_CHANNEL_FLOAT,
    FSF_CHANNEL_DOUBLE
} FSF_CHANNEL_FORMAT;

typedef enum {
    STREAM_TYPE_UNKNOWN = 0,
    STREAM_TYPE_RAW,
    STREAM_TYPE_ACTIVE_BR,
    STREAM_TYPE_PHASE,
    STREAM_TYPE_X,
    STREAM_TYPE_Y,
    STREAM_TYPE_DEPTH,
    STREAM_TYPE_RGB,
    STREAM_TYPE_RAW_NORM,
    STREAM_TYPE_RAW_REAL,
    STREAM_TYPE_RAW_IMAG,
    STREAM_TYPE_RAW_REAL_FILT,
    STREAM_TYPE_RAW_IMAG_FILT,
    STREAM_TYPE_RADIAL,
    STREAM_TYPE_RADIAL_FILT,
    STREAM_TYPE_COMMON_MODE,
    STREAM_TYPE_CONF,
    STREAM_TYPE_VARIANCE,
    STREAM_TYPE_REFLECTIVITY
} STREAM_TYPE;

typedef struct {
    uint32_t HeaderSize;
    uint32_t MagicNumber;
    uint16_t FileFormatMajorVersion;
    uint16_t FileFormatMinorVersion;
    uint32_t FileHeaderSize;
    uint32_t StreamInfoSize;
    uint32_t StreamHeaderSize;
    uint32_t OptionalFileHdrSize;
    uint32_t FileCommentSize;
    uint64_t FrameOffsetInfoLoc;
    uint32_t nFrames;
    uint32_t nStreams;
} FSFFILEHEADER;

typedef struct {
    uint32_t SystemID;
    uint32_t StreamType;
    uint32_t ChannelFormat;
    uint32_t BytesPerPixel;
    uint32_t nRowsPerStream;
    uint32_t nColsPerStream;
    uint32_t OptionalStreamHdrSize;
    uint32_t StreamCommentSize;
    uint32_t CompressionScheme;
} FSFSTREAMINFO;

typedef struct {
    uint32_t TimeStamp;
    uint32_t CompressedStreamSize;
} FSFSTREAMHEADER;

typedef const char *LPCSTR;

bool CreateFSFFile(unsigned int *fileHandle, LPCSTR fileName,
                   unsigned int nStreams, FSFSTREAMINFO *streamInfo,
                   unsigned int optFileHdrSize, unsigned int fileCommSize,
                   unsigned int nFrames);

bool ReadFSFFile(LPCSTR fileName, unsigned int *fileHandle);

bool ReadFileHeader(unsigned int fileHandle, FSFFILEHEADER *fileHeader);

bool ReadStreamInfo(unsigned int fileHandle, unsigned int streamID,
                    FSFSTREAMINFO *streamInfo);

bool WriteOptionalFileHdr(unsigned int fileHandle, void *optionalFileHeader);

bool ReadOptionalFileHdr(unsigned int fileHandle, void *optionalFileHeader);

bool WriteFileComments(unsigned int fileHandle, void *fileComments);

bool ReadFileComments(unsigned int fileHandle, void *fileComments);

bool WriteStream(unsigned int fileHandle, unsigned int frameID,
                 unsigned int streamID, unsigned int timeStampInmsecs,
                 void *optStreamHdr, void *comments, void *frameData);

bool ReadStream(unsigned int fileHandle, unsigned int frameID,
                unsigned int streamID, unsigned int *timeStampInmsecs,
                void *optStreamHdr, void *comments, void *frameData);

bool IsFileOpen(unsigned int fileHandle);

bool Close(unsigned int fileHandle);

unsigned int GetWindowsErrorCode(void);

void SetMaxFrames(unsigned int maxFrames);

#ifdef __cplusplus
}
#endif

#endif // FSF_WRAPPER_H
