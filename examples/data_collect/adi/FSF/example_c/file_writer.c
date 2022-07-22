/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
/**
 * @brief   Writer example with FSF API C wrapper
 *
 * @details This example application uses the FSF API C wrapper to
 *          dummy stream data into an FSF file.
 */
#include <stdlib.h>
#include <fsf_wrapper.h>

#define N_FRAMES    2
#define N_STREAMS   4

int main(void)
{
    const unsigned int systemID = 1;
    const unsigned int nRows = 384;
    const unsigned int nCols = 288;

    FSFSTREAMINFO info[N_STREAMS] = {0};

    //Stream type
    info[0].StreamType = STREAM_TYPE_X;
    info[1].StreamType = STREAM_TYPE_Y;
    info[2].StreamType = STREAM_TYPE_DEPTH;
    info[3].StreamType = STREAM_TYPE_ACTIVE_BR;

    for (int streamId = 0; streamId < N_STREAMS; streamId++)
    {
        info[streamId].BytesPerPixel = sizeof(float);
        info[streamId].ChannelFormat = FSF_CHANNEL_FLOAT;
        info[streamId].SystemID = systemID;
        info[streamId].nColsPerStream = nCols;
        info[streamId].nRowsPerStream = nRows;
    }

    char *fileName = "DataFile.fsf";
    unsigned int fileHandle = 0;
    unsigned int optionalFileHdrSize = 0;

    bool status = CreateFSFFile(&fileHandle, fileName, N_STREAMS, info,
                                optionalFileHdrSize, 0, N_FRAMES);

    unsigned int timeStamp = 0;
    unsigned int frameId = 0;

    float *xFrame = malloc(sizeof(float) * nRows * nCols);
    float *yFrame = malloc(sizeof(float) * nRows * nCols);
    float *zFrame = malloc(sizeof(float) * nRows * nCols);
    float *abFrame = malloc(sizeof(float) * nRows * nCols);

    if (status == true)
    {
        //frame 0
        WriteStream(fileHandle, frameId, 0, timeStamp, 0, 0, xFrame);
        WriteStream(fileHandle, frameId, 1, timeStamp, 0, 0, yFrame);
        WriteStream(fileHandle, frameId, 2, timeStamp, 0, 0, zFrame);
        WriteStream(fileHandle, frameId, 3, timeStamp, 0, 0, abFrame);

        //frame 1
        frameId = 1;
        WriteStream(fileHandle, frameId, 0, timeStamp, 0, 0, xFrame);
        WriteStream(fileHandle, frameId, 1, timeStamp, 0, 0, yFrame);
        WriteStream(fileHandle, frameId, 2, timeStamp, 0, 0, zFrame);
        WriteStream(fileHandle, frameId, 3, timeStamp, 0, 0, abFrame);

        //...

        Close(fileHandle);
    }

    free(xFrame);
    free(yFrame);
    free(zFrame);
    free(abFrame);

    return 0;
}
