/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
/**
 * @brief   FSF file writer example
 *
 * @details This example application uses the FSF parser class to write
 *          dummy stream data into an FSF file.
 */
#include <fsf_common.h>

#define N_FRAMES    2
#define N_STREAMS   4

int main(void)
{
    const unsigned int systemID = 1;
    const unsigned int nRows = 384;
    const unsigned int nCols = 288;

    aditof::FileHeader fileHeader {};
    aditof::StreamInfo info[N_STREAMS] {};
    aditof::Stream stream {};

    const char *fileName = "DataFile.fsf";
    aditof::FSF *fileHandle = new aditof::FSF_Common { aditof::FsfMode::WRITE };
    aditof::FsfStatus status = fileHandle->CreateFsfFile(fileName);

    fileHeader.OptionalFileHdrSize = 0;
    fileHeader.FileCommentSize = 0;
    fileHeader.nFrames = N_FRAMES;
    fileHeader.nStreams = N_STREAMS;

    if (status == aditof::FsfStatus::SUCCESS)
    {
        status = fileHandle->SetFileHeader(fileHeader);
    }

    //Stream type
    info[0].StreamType = static_cast<uint32_t>(aditof::StreamType::STREAM_TYPE_X);
    info[1].StreamType = static_cast<uint32_t>(aditof::StreamType::STREAM_TYPE_Y);
    info[2].StreamType = static_cast<uint32_t>(aditof::StreamType::STREAM_TYPE_DEPTH);
    info[3].StreamType = static_cast<uint32_t>(aditof::StreamType::STREAM_TYPE_ACTIVE_BR);

    for (unsigned int streamId = 0; streamId < N_STREAMS; streamId++)
    {
        info[streamId].SystemID = systemID;
        info[streamId].ChannelFormat = static_cast<uint32_t>(aditof::ChannelFormat::FSF_CHANNEL_FLOAT);
        info[streamId].BytesPerPixel = sizeof(float);
        info[streamId].nRowsPerStream = nRows;
        info[streamId].nColsPerStream = nCols;

        status = fileHandle->SetStreamInfo(streamId, info[streamId]);
    }

    unsigned int timeStamp = 0;
    unsigned int frameId = 0;
    unsigned int streamSize = sizeof(float) * nRows * nCols;

    float* xFrame = new float[nRows * nCols] {};
    float* yFrame = new float[nRows * nCols] {};
    float* zFrame = new float[nRows * nCols] {};
    float* abFrame = new float[nRows * nCols] {};

    stream.streamHeader.TimeStamp = timeStamp;
    stream.streamHeader.CompressedStreamSize = streamSize;

    if (status == aditof::FsfStatus::SUCCESS)
    {
        // Frame 0
        stream.streamData.assign((char *)xFrame, (char *)xFrame + streamSize);
        fileHandle->SetStream(frameId, 0, stream);
        stream.streamData.assign((char *)yFrame, (char *)yFrame + streamSize);
        fileHandle->SetStream(frameId, 1, stream);
        stream.streamData.assign((char *)zFrame, (char *)zFrame + streamSize);
        fileHandle->SetStream(frameId, 2, stream);
        stream.streamData.assign((char *)abFrame, (char *)abFrame + streamSize);
        fileHandle->SetStream(frameId, 3, stream);

        // Frame 1
        frameId = 1;
        stream.streamData.assign((char *)xFrame, (char *)xFrame + streamSize);
        fileHandle->SetStream(frameId, 0, stream);
        stream.streamData.assign((char *)yFrame, (char *)yFrame + streamSize);
        fileHandle->SetStream(frameId, 1, stream);
        stream.streamData.assign((char *)zFrame, (char *)zFrame + streamSize);
        fileHandle->SetStream(frameId, 2, stream);
        stream.streamData.assign((char *)abFrame, (char *)abFrame + streamSize);
        fileHandle->SetStream(frameId, 3, stream);

        // Save and close
        fileHandle->SaveFile();
        fileHandle->CloseFile();
    }

    delete[] xFrame;
    delete[] yFrame;
    delete[] zFrame;
    delete[] abFrame;
}
