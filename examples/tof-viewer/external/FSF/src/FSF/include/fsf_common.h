/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef FSF_COMMON_H
#define FSF_COMMON_H

#include <fstream>
#include <vector>
#include <fsf/fsf.h>

#define DEFAULT_MAX_FRAMES 300UL
#define DEFAULT_MAX_STREAMS 18UL

namespace aditof {

class FSF_Common : public FSF
{
    public:
        FSF_Common(FsfMode mode);

        FSF_Common(FsfMode mode, const unsigned int maxFrames);

        ~FSF_Common();

        virtual FsfStatus CreateFsfFile(const char *pFilename);

        virtual FsfStatus OpenFile(const char *pFilename);

        virtual FsfStatus SaveFile(void);

        virtual FsfStatus CloseFile(void);

        virtual FsfStatus SetFileHeader(FileHeader &fileHeader);

        virtual FsfStatus GetFileHeader(FileHeader &fileHeader);

        virtual FsfStatus SetStreamInfo(const unsigned int streamIndex,
                                        StreamInfo &streamInfo);

        virtual FsfStatus GetStreamInfo(const unsigned int streamIndex,
                                        StreamInfo &streamInfo);

        virtual FsfStatus SetOptionalFileHeader(OptionalFileHeader &optFileHeader);

        virtual FsfStatus GetOptionalFileHeader(OptionalFileHeader &optFileHeader);

        virtual FsfStatus SetFileComment(FileComment &fileComment);

        virtual FsfStatus GetFileComment(FileComment &fileComment);

        virtual FsfStatus SetStream(const unsigned int frameIndex,
                                    const unsigned int streamIndex,
                                    Stream &stream);

        virtual FsfStatus GetStream(const unsigned int frameIndex,
                                    const unsigned int streamIndex,
                                    Stream &stream);

    private:
        virtual FsfStatus CheckFileHeader(void);

        virtual FsfStatus CalculateSizesAndOffset(void);

        virtual FsfStatus CheckIndex(const unsigned int frameIndex,
                                     const unsigned int streamIndex);

    private:
        virtual FsfStatus Open(const char *pFilename);

        virtual FsfStatus Close(void);

        virtual FsfStatus Write(char * const buffer, const size_t length,
                                uint64_t offset);

        virtual FsfStatus Read(char * const buffer, const size_t length,
                               uint64_t offset);

    private:
        const FsfMode   m_mode;
        const uint32_t  m_maxFrames { DEFAULT_MAX_FRAMES };
        const uint32_t  m_maxStreams { DEFAULT_MAX_STREAMS };
        std::fstream    m_file;
        size_t          m_fileSize;

        /* Internal FSF Structures */
        FileHeader                          m_fileHeader;
        std::vector<StreamInfo>             m_streamInfo;
        std::vector<size_t>                 m_streamDataSize;
        std::vector<uint64_t>               m_streamOffset;
        OptionalFileHeader                  m_optFileHeader;
        FileComment                         m_fileComment;
        std::vector<std::vector<Stream>>    m_frames;
        uint64_t                            m_frameSize;
        std::vector<uint64_t>               m_frameOffset;
};

} // namespace aditof

#endif // FSF_COMMON_H
