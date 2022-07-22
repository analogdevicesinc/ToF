/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef FSF_DEFINITIONS_H
#define FSF_DEFINITIONS_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#define FSF_MAGIC_NUMBER            0x00465346  // "FSF"
#define FILE_FORMAT_MAJOR_VERSION   1
#define FILE_FORMAT_MINOR_VERSION   1
#define FILE_HEADER_SIZE            sizeof(FileHeader)
#define STREAM_INFO_SIZE            sizeof(StreamInfo)
#define STREAM_HEADER_SIZE          sizeof(StreamHeader)

namespace aditof {

/**
 * @brief   FSF file mode
 */
enum class FsfMode {
    READ,
    WRITE
};

/**
 * @brief   FSF file processing status
 */
enum class FsfStatus {
    SUCCESS = 0,
    FILE_NOT_OPEN,
    FILE_NOT_CREATED,
    FILE_DOESNOT_EXIST,
    FILE_FORMAT_ERROR,
    FILE_HEADER_ERROR,
    INVALID_OPERATION,
    FAILED = -1
};

/**
 * @brief   FSF file stream data types
 */
enum class StreamType {
    STREAM_TYPE_UNKNOWN,
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
};

enum class ChannelFormat{
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
};

struct FileHeader {
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
};

struct StreamInfo {
    uint32_t SystemID;
    uint32_t StreamType;
    uint32_t ChannelFormat;
    uint32_t BytesPerPixel;
    uint32_t nRowsPerStream;
    uint32_t nColsPerStream;
    uint32_t OptionalStreamHdrSize;
    uint32_t StreamCommentSize;
    uint32_t CompressionScheme;
};

typedef std::string OptionalFileHeader;

typedef std::string FileComment;

struct StreamHeader {
    uint32_t TimeStamp;
    uint32_t CompressedStreamSize;
};

typedef std::string OptionalStreamHeader;

typedef std::string StreamComment;

typedef std::vector<uint8_t> StreamData;

struct Stream {
    StreamHeader streamHeader;
    OptionalStreamHeader optionalStreamHeader;
    StreamComment streamComment;
    StreamData streamData;
};

} // namespace aditof

#endif // FSF_DEFINITIONS_H
