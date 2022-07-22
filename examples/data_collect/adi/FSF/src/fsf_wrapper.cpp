/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
/**
 * @brief   FSF parser utility C wrapper
 */
#include <string.h>
#include <fsf_common.h>
#include <fsf_wrapper.h>

extern "C" {

/*==========  P R I V A T E   T Y P E S  ==========*/

/*
 * FSF usage mode
 */
typedef enum {
    NOT_SET = 0,
    WRITE,
    READ
} FSFMODE;

/*
 * FSF instance handle
 */
typedef struct {
    aditof::FSF *Impl;
    FSFMODE Mode;
    FSFFILEHEADER FileHeader;
    FSFSTREAMINFO *StreamInfo;
} FSFHANDLE;

static FSFHANDLE FSFHandle[MAX_FILES] = {};
static unsigned int m_maxFrames = DEFAULT_MAX_FRAMES;

/*
 * FSF API return status
 */
static aditof::FsfStatus Status;

/*==========  P R I V A T E   F U N C T I O N S  ==========*/

/*
 * @brief   Returns index of available FSF handle
 */
static unsigned int GetFreeHandleID(void) {
    unsigned int handleID;

    for (handleID = 0; handleID < MAX_FILES; handleID++) {
        if ((FSFHandle[handleID].Impl == NULL) &&
            (FSFHandle[handleID].Mode == NOT_SET)) {
            break;
        }
    }

    return handleID;
}

/*
 * @brief   Check for valid FSF handle index
 */
static bool CheckHandleID(unsigned int handleID) {
    return (handleID < MAX_FILES);
}

/*
 * @brief   Return pointer to indexed FSF handle
 */
static FSFHANDLE *GetFSFHandle(unsigned int handleID) {
    if (!CheckHandleID(handleID)) {
        return NULL;
    }
    else {
        return &FSFHandle[handleID];
    }
}

/*==========  P U B L I C   F U N C T I O N S  ==========*/

bool CreateFSFFile(unsigned int *fileHandle, LPCSTR fileName,
                   unsigned int nStreams, FSFSTREAMINFO *streamInfo,
                   unsigned int optFileHdrSize, unsigned int fileCommSize,
                   unsigned int nFrames) {
    FSFHANDLE *hFSF;
    aditof::FileHeader mFileHeader {};
    aditof::StreamInfo mStreamInfo {};
    unsigned int handleID;

    handleID = GetFreeHandleID();
    hFSF = GetFSFHandle(handleID);

    if ((hFSF == NULL) || (fileHandle == NULL) || (streamInfo == NULL)) {
        return false;
    }
    else {
        // Set file handle to be the free handle ID
        *fileHandle = handleID;

        // Set FSF handle
        hFSF->Impl = new aditof::FSF_Common { aditof::FsfMode::WRITE, m_maxFrames };
        hFSF->Mode = NOT_SET;
        hFSF->FileHeader.OptionalFileHdrSize = optFileHdrSize;
        hFSF->FileHeader.FileCommentSize = fileCommSize;
        hFSF->FileHeader.nFrames = nFrames;
        hFSF->FileHeader.nStreams = nStreams;
        hFSF->StreamInfo = streamInfo;
    }

    Status = hFSF->Impl->CreateFsfFile(fileName);

    if (Status != aditof::FsfStatus::SUCCESS) {
        return false;
    }

    memcpy(&mFileHeader, &hFSF->FileHeader, sizeof(FSFFILEHEADER));

    Status = hFSF->Impl->SetFileHeader(mFileHeader);

    if (Status != aditof::FsfStatus::SUCCESS) {
        return false;
    }

    for (unsigned int streamID = 0; streamID < nStreams; streamID++) {
        memcpy(&mStreamInfo, &streamInfo[streamID], sizeof(FSFSTREAMINFO));

        Status = hFSF->Impl->SetStreamInfo(streamID, mStreamInfo);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
    }

    // Set handle to be in use if sucessfully opened file
    hFSF->Mode = WRITE;

    return true;
}

bool ReadFSFFile(LPCSTR fileName, unsigned int *fileHandle) {
    FSFHANDLE *hFSF;
    unsigned int handleID;

    handleID = GetFreeHandleID();
    hFSF = GetFSFHandle(handleID);

    if ((hFSF == NULL) || (fileHandle == NULL)) {
        return false;
    }
    else {
        // Set file handle to be the free handle ID
        *fileHandle = handleID;

        // Set FSF handle
        hFSF->Impl = new aditof::FSF_Common { aditof::FsfMode::READ, m_maxFrames };
        hFSF->Mode = NOT_SET;

        Status = hFSF->Impl->OpenFile(fileName);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            // Set handle to be in use if sucessfully opened file
            hFSF->Mode = READ;

            return true;
        }
    }
}

bool ReadFileHeader(unsigned int fileHandle, FSFFILEHEADER *fileHeader) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::FileHeader mFileHeader;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        Status = hFSF->Impl->GetFileHeader(mFileHeader);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            if (fileHeader != NULL) {
                memcpy(fileHeader, &mFileHeader, sizeof(FSFFILEHEADER));
            }

            return true;
        }
    }
}

bool ReadStreamInfo(unsigned int fileHandle, unsigned int streamID,
                    FSFSTREAMINFO *streamInfo) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::StreamInfo mStreamInfo;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        Status = hFSF->Impl->GetStreamInfo(streamID, mStreamInfo);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            if (streamInfo != NULL) {
                memcpy(streamInfo, &mStreamInfo, sizeof(FSFSTREAMINFO));
            }

            return true;
        }
    }
}

bool WriteOptionalFileHdr(unsigned int fileHandle, void *optionalFileHeader) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::OptionalFileHeader optFileHeader;
    unsigned int optFileHdrSize;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        optFileHdrSize = hFSF->FileHeader.OptionalFileHdrSize;

        if (optionalFileHeader != NULL) {
            optFileHeader.assign((char *)optionalFileHeader, optFileHdrSize);
        }


        Status = hFSF->Impl->SetOptionalFileHeader(optFileHeader);

        return (Status == aditof::FsfStatus::SUCCESS);
    }
}

bool ReadOptionalFileHdr(unsigned int fileHandle, void *optionalFileHeader) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::OptionalFileHeader optFileHeader;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        Status = hFSF->Impl->GetOptionalFileHeader(optFileHeader);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            if (optionalFileHeader != NULL) {
                memcpy(optionalFileHeader, optFileHeader.data(),
                       optFileHeader.size());
            }

            return true;
        }
    }
}

bool WriteFileComments(unsigned int fileHandle, void *fileComments) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::FileComment fileComment;
    unsigned int fileCommSize;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        fileCommSize = hFSF->FileHeader.FileCommentSize;

        if (fileComments != NULL) {
            fileComment.assign((char *)fileComments, fileCommSize);
        }

        Status = hFSF->Impl->SetFileComment(fileComment);

        return (Status == aditof::FsfStatus::SUCCESS);
    }
}

bool ReadFileComments(unsigned int fileHandle, void *fileComments) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::FileComment fileComment;

    if ((hFSF == NULL) || (hFSF->Mode == NOT_SET)) {
        return false;
    }
    else {
        Status = hFSF->Impl->GetFileComment(fileComment);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            if (fileComments != NULL) {
                memcpy(fileComments, fileComment.data(), fileComment.size());
            }

            return true;
        }
    }
}

bool WriteStream(unsigned int fileHandle, unsigned int frameID,
                 unsigned int streamID, unsigned int timeStampInmsecs,
                 void *optStreamHdr, void *comments, void *frameData) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    FSFSTREAMINFO mStreamInfo;
    aditof::Stream stream;

    if (hFSF == NULL || hFSF->Mode == NOT_SET) {
        return false;
    }
    else {
        mStreamInfo = hFSF->StreamInfo[streamID];

        stream.streamHeader.TimeStamp = timeStampInmsecs;

        stream.streamHeader.CompressedStreamSize = mStreamInfo.BytesPerPixel
                                                 * mStreamInfo.nRowsPerStream
                                                 * mStreamInfo.nColsPerStream;

        if (optStreamHdr != NULL) {
            stream.optionalStreamHeader.assign((char *)optStreamHdr,
                                               mStreamInfo.OptionalStreamHdrSize);
        }

        if (comments != NULL) {
            stream.streamComment.assign((char *)comments,
                                        mStreamInfo.StreamCommentSize);
        }

        if (frameData != NULL) {
            stream.streamData.assign((char *)frameData,
                                     (char *)frameData +
                                     stream.streamHeader.CompressedStreamSize);
        }

        Status = hFSF->Impl->SetStream(frameID, streamID, stream);

        return (Status == aditof::FsfStatus::SUCCESS);
    }
}

bool ReadStream(unsigned int fileHandle, unsigned int frameID,
                unsigned int streamID, unsigned int *timeStampInmsecs,
                void *optStreamHdr, void *comments, void *frameData) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);
    aditof::Stream stream;

    if (hFSF == NULL || hFSF->Mode == NOT_SET) {
        return false;
    }
    else {
        Status = hFSF->Impl->GetStream(frameID, streamID, stream);

        if (Status != aditof::FsfStatus::SUCCESS) {
            return false;
        }
        else {
            *timeStampInmsecs = stream.streamHeader.TimeStamp;

            if (optStreamHdr != NULL) {
                memcpy(optStreamHdr, stream.optionalStreamHeader.data(),
                       stream.optionalStreamHeader.size());
            }

            if (comments != NULL) {
                memcpy(comments, stream.streamComment.data(),
                       stream.streamComment.size());
            }

            if (frameData != NULL) {
                memcpy(frameData, stream.streamData.data(),
                       stream.streamData.size());
            }

            return true;
        }
    }
}

bool IsFileOpen(unsigned int fileHandle) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);

    if (hFSF == NULL || hFSF->Mode == NOT_SET) {
        return false;
    }
    else {
        return true;
    }
}

bool Close(unsigned int fileHandle) {
    FSFHANDLE *hFSF = GetFSFHandle(fileHandle);

    if (hFSF == NULL) {
        return false;
    }
    else if (hFSF->Mode == NOT_SET) {
        delete hFSF->Impl;
        hFSF->Impl = NULL;

        return true;
    }
    else {
        if (hFSF->Mode == WRITE) {
            Status = hFSF->Impl->SaveFile();
            hFSF->Impl->CloseFile();
        }
        else {
            Status = hFSF->Impl->CloseFile();
        }

        // Free the handle for next use
        delete hFSF->Impl;
        hFSF->Impl = NULL;
        hFSF->Mode = NOT_SET;

        return (Status == aditof::FsfStatus::SUCCESS);
    }
}

unsigned int GetWindowsErrorCode(void) {
    unsigned int errorCode;

    switch (Status) {
        case aditof::FsfStatus::SUCCESS:
            errorCode = SUCCESS;
        break;

        case aditof::FsfStatus::FILE_NOT_OPEN:
            errorCode = FILE_NOT_OPEN;
        break;

        case aditof::FsfStatus::FILE_NOT_CREATED:
            errorCode = FILE_NOT_CREATED;
        break;

        case aditof::FsfStatus::FILE_DOESNOT_EXIST:
            errorCode = FILE_DOESNOT_EXIST;
        break;

        case aditof::FsfStatus::FILE_FORMAT_ERROR:
            errorCode = FILE_FORMAT_ERROR;
        break;

        case aditof::FsfStatus::FILE_HEADER_ERROR:
            errorCode = FILE_HEADER_ERROR;
        break;

        case aditof::FsfStatus::INVALID_OPERATION:
            errorCode = INVALID_OPERATION;
        break;

        case aditof::FsfStatus::FAILED:
            errorCode = FAILED;
        break;

        default:
            errorCode = FAILED;
        break;
    }

    return errorCode;
}

void SetMaxFrames(unsigned int maxFrames) {
    m_maxFrames = maxFrames;
}

} // extern "C"
