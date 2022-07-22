/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <iostream>
#include <fsf_common.h>

namespace aditof {

FSF_Common::FSF_Common(FsfMode mode)
    :m_mode(mode) {}

FSF_Common::FSF_Common(FsfMode mode, const unsigned int maxFrames)
    :m_mode(mode), m_maxFrames(maxFrames) {}

FSF_Common::~FSF_Common() {
    FSF_Common::Close();
}

FsfStatus FSF_Common::CreateFsfFile(const char *pFilename) {
    FsfStatus status;

    // CreateFsfFile only in WRITE mode
    if (m_mode != FsfMode::WRITE) {
        std::cout << "CreateFsfFile: Invalid operation in READ mode." << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else {
        status = FSF_Common::Open(pFilename);

        if (status != FsfStatus::SUCCESS) {
            std::cout << "CreateFsfFile: Error opening " << pFilename
                      << " file for writing." << std::endl;

            FSF_Common::Close();
            status = FsfStatus::FILE_NOT_CREATED;
        }
    }

    return status;
}

FsfStatus FSF_Common::OpenFile(const char *pFilename) {
    FsfStatus status;

    // OpenFile only in READ mode
    if (m_mode != FsfMode::READ)
    {
        std::cout << "OpenFile: Invalid operation in WRITE mode." << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else
    {
        status = FSF_Common::Open(pFilename);

        if (status != FsfStatus::SUCCESS) {
            std::cout << "OpenFile: Error opening " << pFilename
                      << " file for reading." << std::endl;

            status = FsfStatus::FILE_DOESNOT_EXIST;
        }
        else if (m_fileSize == 0) {
            std::cout << "OpenFile: Empty file." << std::endl;

            FSF_Common::Close();
            status = FsfStatus::FILE_FORMAT_ERROR;
        }
        else {
            // Check for valid file format
            uint32_t magicNumber;
            char * buffer = reinterpret_cast<char *>(&magicNumber);

            status = FSF_Common::Read(buffer, sizeof(uint32_t), sizeof(uint32_t));

            if (magicNumber != FSF_MAGIC_NUMBER) {
                std::cout << "OpenFile: Invalid file format." << std::endl;

                FSF_Common::Close();
                status = FsfStatus::FILE_FORMAT_ERROR;
            }
            else {
                status = FsfStatus::SUCCESS;
            }
        }
    }

    return status;
}

FsfStatus FSF_Common::SaveFile(void) {
    FsfStatus status;
    uint64_t offset;
    size_t length;
    char * buffer;
    unsigned int frameIdx;
    unsigned int streamIdx;

    // SaveFile only in WRITE mode
    if (m_mode != FsfMode::WRITE) {
        std::cout << "SaveFile: Invalid operation in READ mode." << std::endl;

        return FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "SaveFile: Invalid file header." << std::endl;

       return FsfStatus::FILE_HEADER_ERROR;
    }
    else {
        // Compute for internal sizes and offset values
        if (m_frameSize == 0) {
            FSF_Common::CalculateSizesAndOffset();
        }

        // Write file header to file
        offset = 0;
        length = FILE_HEADER_SIZE;
        buffer = reinterpret_cast<char *>(&m_fileHeader);

        status = FSF_Common::Write(buffer, length, offset);
        offset += length;

        if (status != FsfStatus::SUCCESS) {
            std::cout << "SaveFile: Error writing file header."
                      << std::endl;
            return status;
        }

        // Write stream info to file
        for (StreamInfo streamInfo: m_streamInfo) {
            length = STREAM_INFO_SIZE;
            buffer = reinterpret_cast<char *>(&streamInfo);

            status = FSF_Common::Write(buffer, length, offset);
            offset += length;

            if (status != FsfStatus::SUCCESS) {
                std::cout << "SaveFile: Error writing stream info."
                          << std::endl;
                return status;
            }
        }

        // Write optional file header to file
        length = m_fileHeader.OptionalFileHdrSize;

        if (m_optFileHeader.size() != 0) {
            buffer = const_cast<char *>(m_optFileHeader.data());
            status = FSF_Common::Write(buffer, length, offset);
        }

        m_optFileHeader.clear();
        std::string().swap(m_optFileHeader);
        offset += length;

        if (status != FsfStatus::SUCCESS) {
            std::cout << "SaveFile: Error writing optional "
                      << "file header." << std::endl;
            return status;
        }

        // Write file comment to file
        length = m_fileHeader.FileCommentSize;

        if (m_fileComment.size() != 0) {
            buffer = const_cast<char *>(m_fileComment.data());
            status = FSF_Common::Write(buffer, length, offset);
        }

        m_fileComment.clear();
        std::string().swap(m_fileComment);
        offset += length;

        if (status != FsfStatus::SUCCESS) {
            std::cout << "SaveFile: Error writing file comment."
                      << std::endl;
            return status;
        }

        // Write frames to file
        for (frameIdx = 0; frameIdx < m_fileHeader.nFrames; frameIdx++) {
            for (streamIdx = 0; streamIdx < m_fileHeader.nStreams; streamIdx++) {
                StreamInfo &streamInfo = m_streamInfo[streamIdx];
                Stream &stream = m_frames[frameIdx][streamIdx];

                // Write stream header to file
                length = sizeof(StreamHeader);
                buffer = reinterpret_cast<char *>(&stream.streamHeader);

                status = FSF_Common::Write(buffer, length, offset);
                offset += length;

                if (status != FsfStatus::SUCCESS) {
                    std::cout << "SaveFile: Error writing stream header."
                              << std::endl;
                    return status;
                }

                // Write optional stream header string to file
                length = streamInfo.OptionalStreamHdrSize;

                if (stream.optionalStreamHeader.size() != 0) {
                    buffer = const_cast<char *>
                             (stream.optionalStreamHeader.data());
                    status = FSF_Common::Write(buffer, length, offset);
                }

                stream.optionalStreamHeader.clear();
                std::string().swap(stream.optionalStreamHeader);
                offset += length;

                if (status != FsfStatus::SUCCESS) {
                    std::cout << "SaveFile: Error writing optional "
                              << "stream header." << std::endl;
                    return status;
                }

                // Write optional stream comment string to file
                length = streamInfo.StreamCommentSize;

                if (stream.streamComment.size() != 0) {
                    buffer = const_cast<char *>(stream.streamComment.data());
                    status = FSF_Common::Write(buffer, length, offset);
                }

                stream.streamComment.clear();
                std::string().swap(stream.streamComment);
                offset += length;

                if (status != FsfStatus::SUCCESS) {
                    std::cout << "SaveFile: Error writing stream comment."
                              << std::endl;
                    return status;
                }

                // Write stream data to file
                length = m_streamDataSize[streamIdx];

                if (stream.streamData.size() != 0) {
                    buffer = reinterpret_cast<char *>(stream.streamData.data());
                    status = FSF_Common::Write(buffer, length, offset);
                }

                stream.streamData.clear();
                std::vector<uint8_t>().swap(stream.streamData);
                offset += length;

                if (status != FsfStatus::SUCCESS) {
                    std::cout << "SaveFile: Error writing stream data."
                              << std::endl;
                    return status;
                }
            }
        }

        for (uint64_t frameOffset: m_frameOffset) {
            length = sizeof(frameOffset);
            buffer = reinterpret_cast<char *>(&frameOffset);

            //  Write FrameOffset info to file
            status = FSF_Common::Write(buffer, length, offset);
            offset += length;

            if (status != FsfStatus::SUCCESS) {
                std::cout << "SaveFile: Error writing frame offset info."
                          << std::endl;
                return status;
            }
        }
    }

    return FsfStatus::SUCCESS;
}

FsfStatus FSF_Common::CloseFile(void) {
    return FSF_Common::Close();
}

FsfStatus FSF_Common::SetFileHeader(FileHeader &fileHeader) {
    FsfStatus status;

    if (m_mode != FsfMode::WRITE) {
        std::cout << "SetFileHeader: Invalid operation in READ mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else {
        m_fileHeader.HeaderSize = FILE_HEADER_SIZE
                                + (fileHeader.nStreams * STREAM_INFO_SIZE);

        // Set file header constants
        m_fileHeader.MagicNumber = FSF_MAGIC_NUMBER;
        m_fileHeader.FileFormatMajorVersion = FILE_FORMAT_MAJOR_VERSION;
        m_fileHeader.FileFormatMinorVersion = FILE_FORMAT_MINOR_VERSION;
        m_fileHeader.FileHeaderSize = FILE_HEADER_SIZE;
        m_fileHeader.StreamInfoSize = STREAM_INFO_SIZE;
        m_fileHeader.StreamHeaderSize = STREAM_HEADER_SIZE;

        // Set file header configuration
        m_fileHeader.OptionalFileHdrSize = fileHeader.OptionalFileHdrSize;
        m_fileHeader.FileCommentSize = fileHeader.FileCommentSize;
        m_fileHeader.nFrames = fileHeader.nFrames;
        m_fileHeader.nStreams = fileHeader.nStreams;

        if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
            std::cout << "SetFileHeader: Invalid file header." << std::endl;

            status = FsfStatus::FILE_HEADER_ERROR;
        }
        else {
            // Resize StreamInfo vector to fit nStreams
            m_streamInfo.resize(m_fileHeader.nStreams);

            // Resize Frames vector to fit nFrames
            m_frames.resize(m_fileHeader.nFrames);

            // Resize each Frame to fit nStreams
            for (unsigned int index = 0; index < m_fileHeader.nFrames; index++) {
                m_frames[index].resize(m_fileHeader.nStreams);
            }

            // Initialize frame size to 0
            m_frameSize = 0;

            status = FsfStatus::SUCCESS;
        }
    }

    return status;
}

FsfStatus FSF_Common::GetFileHeader(FileHeader &fileHeader) {
    FsfStatus status;
    size_t length;
    char * buffer;

    if (m_mode != FsfMode::READ) {
        std::cout << "GetFileHeader: Invalid operation in WRITE mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else {
        length = FILE_HEADER_SIZE;
        buffer = reinterpret_cast<char *>(&m_fileHeader);

        // Get FileHeader from file
        status = FSF_Common::Read(buffer, length, 0);

        if (status != FsfStatus::SUCCESS) {
            std::cout << "GetFileHeader: Error retrieving FileHeader."
                      << std::endl;
        }
        else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
            std::cout << "GetFileHeader: Invalid file header." << std::endl;

            status = FsfStatus::FILE_HEADER_ERROR;
        }
        else {
            fileHeader = m_fileHeader;

            // Resize StreamInfo vector to fit nStreams
            m_streamInfo.resize(m_fileHeader.nStreams);

            // Initialize frame size to 0
            m_frameSize = 0;
        }
    }

    return status;
}

FsfStatus FSF_Common::SetStreamInfo(const unsigned int streamIndex,
                                    StreamInfo &streamInfo) {
    FsfStatus status;

    if (m_mode != FsfMode::WRITE) {
        std::cout << "SetStreamInfo: Invalid operation in READ mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "SetStreamInfo: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (streamIndex >= m_fileHeader.nStreams) {
        std::cout << "SetStreamInfo: Cannot set StreamInfo "
                  << "at StreamInfo[" << streamIndex << "] "
                  << "with " << m_fileHeader.nStreams << " nStreams."
                  << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        // Set stream info for the indexed location
        m_streamInfo[streamIndex] = streamInfo;

        status = FsfStatus::SUCCESS;
    }

    return status;
}

FsfStatus FSF_Common::GetStreamInfo(const unsigned int streamIndex,
                                    StreamInfo &streamInfo) {
    FsfStatus status;
    uint64_t offset;
    size_t length;
    char * buffer;

    if (m_mode != FsfMode::READ) {
        std::cout << "GetStreamInfo: Invalid operation in WRITE mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "GetStreamInfo: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (streamIndex >= m_fileHeader.nStreams) {
        std::cout << "GetStreamInfo: Cannot get StreamInfo "
                  << "at StreamInfo[" << streamIndex << "] "
                  << "with " << m_fileHeader.nStreams << " nStreams."
                  << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        offset = FILE_HEADER_SIZE + (streamIndex * STREAM_INFO_SIZE);
        length = STREAM_INFO_SIZE;
        buffer = reinterpret_cast<char *>(&m_streamInfo[streamIndex]);

        // Get StreamInfo from FSF file
        status = FSF_Common::Read(buffer, length, offset);

        if (status == FsfStatus::SUCCESS) {
            streamInfo = m_streamInfo[streamIndex];
        }
        else {
            std::cout << "GetStreamInfo: Error retrieving StreamInfo."
                      << std::endl;
        }
    }

    return status;
}

FsfStatus FSF_Common::SetOptionalFileHeader(OptionalFileHeader &optFileHeader) {
    FsfStatus status;

    if (m_mode != FsfMode::WRITE) {
        std::cout << "SetOptionalFileHeader: Invalid operation in READ mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "SetOptionalFileHeader: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (m_fileHeader.OptionalFileHdrSize != optFileHeader.size()) {
        std::cout << "SetOptionalFileHeader: Invalid optional file "
                  << "header string size." << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        // Set optional file header only when OptionalFileHdrSize is non-zero
        if (m_fileHeader.OptionalFileHdrSize != 0) {
            m_optFileHeader = optFileHeader.data();
        }

        status = FsfStatus::SUCCESS;
    }

    return status;
}

FsfStatus FSF_Common::GetOptionalFileHeader(OptionalFileHeader &optFileHeader) {
    FsfStatus status;
    uint64_t offset;
    size_t length;
    char * buffer;

    if (m_mode != FsfMode::READ) {
        std::cout << "GetOptionalFileHeader: Invalid operation in WRITE mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "GetOptionalFileHeader: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (m_fileHeader.OptionalFileHdrSize != 0) {
        offset = FILE_HEADER_SIZE
               + (m_fileHeader.nStreams * STREAM_INFO_SIZE);
        length = m_fileHeader.OptionalFileHdrSize;
        buffer = new char [length];

        // Get OptionalFileHeader from file
        status = FSF_Common::Read(buffer, length, offset);

        if (status == FsfStatus::SUCCESS) {
            optFileHeader.assign(buffer, length);
        }
        else {
            std::cout << "GetOptionalFileHeader: Error retrieving optional "
                      << "file header." << std::endl;
        }

        delete[] buffer;
    }
    else {
        status = FsfStatus::SUCCESS;
    }

    return status;
}

FsfStatus FSF_Common::SetFileComment(FileComment &fileComment) {
    FsfStatus status;

    if (m_mode != FsfMode::WRITE) {
        std::cout << "SetFileComment: Invalid operation in READ mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "SetFileComment: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (m_fileHeader.FileCommentSize != fileComment.size()) {
        std::cout << "SetFileComment: Invalid file comment string size."
                  << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        // Set file comment only when FileCommentSize is non-zero
        if (m_fileHeader.FileCommentSize != 0) {
            m_fileComment = fileComment.data();
        }

        status = FsfStatus::SUCCESS;
    }

    return status;
}

FsfStatus FSF_Common::GetFileComment(FileComment &fileComment) {
    FsfStatus status;
    uint64_t offset;
    size_t length;
    char * buffer;

    if (m_mode != FsfMode::READ) {
        std::cout << "GetFileComment: Invalid operation in WRITE mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "GetFileComment: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (m_fileHeader.FileCommentSize != 0) {
        offset = FILE_HEADER_SIZE
               + (m_fileHeader.nStreams * STREAM_INFO_SIZE)
               + m_fileHeader.OptionalFileHdrSize;
        length = m_fileHeader.FileCommentSize;
        buffer = new char [length];

        // Get file comment from file
        status = FSF_Common::Read(buffer, length, offset);

        if (status == FsfStatus::SUCCESS) {
            fileComment.assign(buffer, length);
        }
        else {
            std::cout << "GetFileComment: Error retrieving file comment."
                      << std::endl;
        }

        delete[] buffer;
    }
    else {
        status = FsfStatus::SUCCESS;
    }

    return status;
}

FsfStatus FSF_Common::SetStream(const unsigned int frameIndex,
                                const unsigned int streamIndex,
                                Stream &stream) {
    FsfStatus status;

    if (m_mode != FsfMode::WRITE) {
        std::cout << "SetStream: Invalid operation in READ mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "SetStream: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (FSF_Common::CheckIndex(frameIndex, streamIndex)
             != FsfStatus::SUCCESS) {
        std::cout << "SetStream: Cannot set stream "
                  << "at Frame[" << frameIndex << "] "
                  << "Stream[" << streamIndex << "] "
                  << "with " << m_fileHeader.nFrames << " nFrames "
                  << "and " << m_fileHeader.nStreams << " nStreams."
                  << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        // Compute for internal sizes and offset values
        if (m_frameSize == 0) {
            FSF_Common::CalculateSizesAndOffset();
        }

        // Select stream
        StreamInfo &streamInfo = m_streamInfo[streamIndex];
        Stream &m_stream = m_frames[frameIndex][streamIndex];

        if (streamInfo.OptionalStreamHdrSize !=
            stream.optionalStreamHeader.size()) {
            std::cout << "SetStream: Invalid optional stream header "
                      << "string size." << std::endl;

            status = FsfStatus::FAILED;
        }
        else if (streamInfo.StreamCommentSize != stream.streamComment.size()) {
            std::cout << "SetStream: Invalid stream comment string size."
                      << std::endl;

            status = FsfStatus::FAILED;
        }
        else if (m_streamDataSize[streamIndex] != stream.streamData.size()) {
            std::cout << "SetStream: Invalid stream data size."
                      << std::endl;

            status = FsfStatus::FAILED;
        }
        else {
            // Set stream header struct
            m_stream.streamHeader = stream.streamHeader;

            // Set optional stream header string
            if (streamInfo.OptionalStreamHdrSize!= 0) {
                m_stream.optionalStreamHeader.assign(stream.optionalStreamHeader);
            }

            // Set stream comment string
            if (streamInfo.StreamCommentSize != 0) {
                m_stream.streamComment.assign(stream.streamComment);
            }

            // Set stream data bytes
            if (m_streamDataSize[streamIndex] != 0) {
                m_stream.streamData = stream.streamData;
            }

            status = FsfStatus::SUCCESS;
        }
    }

    return status;
}

FsfStatus FSF_Common::GetStream(const unsigned int frameIndex,
                                const unsigned int streamIndex,
                                Stream &stream) {
    FsfStatus status;
    uint64_t offset;
    size_t length;
    char * buffer;

    if (m_mode != FsfMode::READ) {
        std::cout << "GetStream: Invalid operation in WRITE mode."
                  << std::endl;

        status = FsfStatus::INVALID_OPERATION;
    }
    else if (FSF_Common::CheckFileHeader() != FsfStatus::SUCCESS) {
        std::cout << "GetStream: Invalid file header." << std::endl;

        status = FsfStatus::FILE_HEADER_ERROR;
    }
    else if (FSF_Common::CheckIndex(frameIndex, streamIndex)
             != FsfStatus::SUCCESS) {
        std::cout << "GetStream: Cannot get stream "
                  << "at Frame[" << frameIndex << "] "
                  << "Stream[" << streamIndex << "] "
                  << "with " << m_fileHeader.nFrames << " nFrames "
                  << "and " << m_fileHeader.nStreams << " nStreams."
                  << std::endl;

        status = FsfStatus::FAILED;
    }
    else {
        // Compute for internal sizes and offset values
        if (m_frameSize == 0) {
            FSF_Common::CalculateSizesAndOffset();
        }

        // Get stream info
        StreamInfo &streamInfo = m_streamInfo[streamIndex];

        offset = m_frameOffset[frameIndex] + m_streamOffset[streamIndex];
        length = sizeof(StreamHeader);
        buffer = reinterpret_cast<char *>(&stream.streamHeader);

        // Get stream header struct from file
        status = FSF_Common::Read(buffer, length, offset);

        if (status != FsfStatus::SUCCESS) {
            std::cout << "GetStream: Error retrieving stream header."
                      << std::endl;
        }

        if (streamInfo.OptionalStreamHdrSize != 0) {
            // Use temp buffer for optional stream header string
            offset += length;
            length = streamInfo.OptionalStreamHdrSize;
            buffer = new char [length];

            // Get optional stream header string from file
            status = FSF_Common::Read(buffer, length, offset);

            if (status != FsfStatus::SUCCESS) {
                std::cout << "GetStream: Error retrieving optional stream header."
                          << std::endl;
            }
            else {
                stream.optionalStreamHeader.assign(buffer, length);
            }

            delete[] buffer;
        }

        if (streamInfo.StreamCommentSize != 0) {
            // Use temp buffer for optional stream comment string
            offset += length;
            length = streamInfo.StreamCommentSize;
            buffer = new char [length];

            // Get stream comment string
            status = FSF_Common::Read(buffer, length, offset);

            if (status != FsfStatus::SUCCESS) {
                std::cout << "GetStream: Error retrieving stream comment."
                          << std::endl;
            }
            else {
                stream.streamComment.assign(buffer, length);
            }

            delete[] buffer;
        }

        if (m_streamDataSize[streamIndex] != 0) {
            // Use temp buffer for stream data bytes
            offset += length;
            length = m_streamDataSize[streamIndex];
            buffer = new char [length] {0};

            // Get stream data bytes
            status = FSF_Common::Read(buffer, length, offset);

            if (status != FsfStatus::SUCCESS) {
                std::cout << "GetStream: Error retrieving stream data."
                          << std::endl;
            }
            else {
                stream.streamData.assign(buffer, buffer + length);
            }

            delete[] buffer;
        }
    }

    return status;
}

/*==========  P R I V A T E   M E T H O D S  ==========*/
/*
 * @brief   Check file header validity.
 */
FsfStatus FSF_Common::CheckFileHeader(void) {
    if ((m_fileHeader.MagicNumber != FSF_MAGIC_NUMBER)
        || (m_fileHeader.FileFormatMajorVersion != FILE_FORMAT_MAJOR_VERSION)
        || (m_fileHeader.FileFormatMinorVersion != FILE_FORMAT_MINOR_VERSION)
        || (m_fileHeader.FileHeaderSize != FILE_HEADER_SIZE)
        || (m_fileHeader.StreamInfoSize != STREAM_INFO_SIZE)
        || (m_fileHeader.StreamHeaderSize != STREAM_HEADER_SIZE)
        || (m_fileHeader.nFrames > m_maxFrames)
        || (m_fileHeader.nStreams > m_maxStreams)) {
        return FsfStatus::FAILED;
    }
    else {
        return FsfStatus::SUCCESS;
    }
}

/*
 * @brief   Calculate sizes and offset base on file header and stream info.
 */
FsfStatus FSF_Common::CalculateSizesAndOffset(void) {
    size_t frameOffsetBase = 0;
    size_t streamHeaderSize = 0;

    // Resize vectors to fit the number of frames and streams
    m_frameOffset.resize(m_fileHeader.nFrames);
    m_streamOffset.resize(m_fileHeader.nStreams);
    m_streamDataSize.resize(m_fileHeader.nStreams);

    // Calculate stream sizes and offsets
    m_frameSize = 0;
    for (unsigned int index = 0; index < m_fileHeader.nStreams; index++) {
        StreamInfo streamInfo = m_streamInfo[index];

        // Total stream header size
        streamHeaderSize = sizeof(StreamHeader)
                         + streamInfo.OptionalStreamHdrSize
                         + streamInfo.StreamCommentSize;

        // Stream data size per stream info
        m_streamDataSize[index] = streamInfo.BytesPerPixel
                                * streamInfo.nRowsPerStream
                                * streamInfo.nColsPerStream;

        // Current stream offset is equal to the previous iteration framesize
        m_streamOffset[index] = m_frameSize;

        // Total frame size is equal to cummulative stream sizes
        m_frameSize += (streamHeaderSize + m_streamDataSize[index]);
    }

    if (m_mode == FsfMode::WRITE) {
        // Calculate base frame offset
        frameOffsetBase = FILE_HEADER_SIZE +
                          (STREAM_INFO_SIZE * m_fileHeader.nStreams) +
                          m_fileHeader.OptionalFileHdrSize +
                          m_fileHeader.FileCommentSize;

        // Calculate frame offset from base
        for (unsigned int index = 0; index < m_fileHeader.nFrames; index++) {
            m_frameOffset[index] = frameOffsetBase + (index * m_frameSize);
        }

        // Update Frame offset info location
        m_fileHeader.FrameOffsetInfoLoc = m_frameOffset[m_fileHeader.nFrames - 1]
                                        + m_frameSize;
    }
    else {
        uint64_t offset = m_fileHeader.FrameOffsetInfoLoc;
        size_t length = sizeof(uint64_t);
        char * buffer;

        for (unsigned int index = 0; index < m_fileHeader.nFrames; index++) {
            buffer = reinterpret_cast<char *>(&m_frameOffset[index]);

            // Get frame offset info
            FSF_Common::Read(buffer, length, offset);
            offset += length;
        }
    }

    return FsfStatus::SUCCESS;
}

/*
 * @brief   Check for index validity.
 */
FsfStatus FSF_Common::CheckIndex(const unsigned int frameIndex,
                                 const unsigned int streamIndex) {
    if ((frameIndex < m_fileHeader.nFrames) &&
        (streamIndex < m_fileHeader.nStreams)) {
        return FsfStatus::SUCCESS;
    }
    else {
        return FsfStatus::FAILED;
    }
}

/*
 * @brief   Open file for writing or reading.
 */
FsfStatus FSF_Common::Open(const char *pFilename) {
    // Open file
    if (m_mode == FsfMode::WRITE) {
        m_file.open(pFilename, std::ios::out | std::ios::binary | std::ios::ate);
        m_fileSize =  static_cast<size_t>(m_file.tellp());
        m_file.seekp(0);
    }
    else {
        m_file.open(pFilename, std::ios::in | std::ios::binary | std::ios::ate);
        m_fileSize =  static_cast<size_t>(m_file.tellg());
        m_file.seekg(0);
    }

    // Check that file opened successfully
    if (m_file.is_open()) {
        return FsfStatus::SUCCESS;
    }
    else {
        return FsfStatus::FAILED;
    }
}

/*
 * @brief   Close file.
 */
FsfStatus FSF_Common::Close(void) {
    if (m_file.is_open()) {
        m_file.flush();
        m_file.close();
    }

    return FsfStatus::SUCCESS;
}

/*
 * @brief   Write to file.
 *
 * @note    Method intended for continuously appending data to file.
 */
FsfStatus FSF_Common::Write(char * const buffer, const size_t length,
                            uint64_t offset) {
    size_t writeCount;

    if (m_file.is_open()) {
        m_file.seekp(offset);
        writeCount = static_cast<size_t>(m_file.tellp());
        m_file.write(buffer, length);
        writeCount = static_cast<size_t>(m_file.tellp()) - writeCount;

        if (writeCount == length) {
            return FsfStatus::SUCCESS;
        }
        else {
            return FsfStatus::FAILED;
        }
    }
    else {
        return FsfStatus::FILE_NOT_OPEN;
    }
}

/*
 * @brief   Read from file.
 */
FsfStatus FSF_Common::Read(char * const buffer, const size_t length,
                           uint64_t offset) {
    size_t readCount;

    if (m_file.is_open()) {
        m_file.seekg(offset);
        m_file.read(buffer, length);

        readCount = static_cast<size_t>(m_file.gcount());

        if (readCount == length) {
            return FsfStatus::SUCCESS;
        }
        else {
            return FsfStatus::FAILED;
        }
    }
    else {
        return FsfStatus::FILE_NOT_OPEN;
    }
}

} // namespace aditof
