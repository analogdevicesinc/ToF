/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
/**
 * @brief   FSF parser loop back example with FSF API C wrapper
 *
 * @details This example application uses the FSF API C wrapper to
 *          read and existing FSF file and write the contents to a new file.
 *          The resulting files should be the same.
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fsf_wrapper.h>

#define N_FRAMES    10
#define N_STREAMS   9

void printFileHeader(FSFFILEHEADER *fileHeader);
void printStreamInfo(FSFSTREAMINFO *streamInfo);
void printStringBuffer(const char *p, size_t s);

int main(int argc, char *argv[])
{
    if (argc != 3) {
        printf("Please provide path to input and output FSF files.\n\n");
        printf("Example usage:\n");
        printf("   file_loopback_c.exe <Read_FSF_file> <Write_FSF_file>\n");

        return -1;
    }

    const char *readFile = argv[1];
    const char *writeFile = argv[2];

    FSFFILEHEADER fileHeader = {0};
    FSFSTREAMINFO streamInfo[N_STREAMS] = {0};

    // Create platform agnostic FSF parser objects
    unsigned int readFileHandle = 0;
    unsigned int writeFileHandle = 0;

    ReadFSFFile(readFile, &readFileHandle);
    ReadFileHeader(readFileHandle, &fileHeader);
    printFileHeader(&fileHeader);

    char *optFileHeader = malloc(fileHeader.OptionalFileHdrSize);
    char *fileComment  = malloc(fileHeader.FileCommentSize);

    for (unsigned int streamID = 0; streamID < fileHeader.nStreams; streamID++) {
        ReadStreamInfo(readFileHandle, streamID, &streamInfo[streamID]);
        printStreamInfo(&streamInfo[streamID]);
    }

    CreateFSFFile(&writeFileHandle, writeFile, fileHeader.nStreams, streamInfo,
                  fileHeader.OptionalFileHdrSize, fileHeader.FileCommentSize,
                  fileHeader.nFrames);

    ReadOptionalFileHdr(readFileHandle, optFileHeader);
    WriteOptionalFileHdr(writeFileHandle, optFileHeader);

    printStringBuffer(optFileHeader, fileHeader.OptionalFileHdrSize);
    free(optFileHeader);

    ReadFileComments(readFileHandle, fileComment);
    WriteFileComments(writeFileHandle, fileComment);

    printStringBuffer(fileComment, fileHeader.FileCommentSize);
    free(fileComment);

    for (unsigned int frameID = 0; frameID < fileHeader.nFrames; frameID++) {
        for (unsigned int streamID = 0; streamID < fileHeader.nStreams; streamID++) {
            unsigned int timeStampInmsecs;

            char *optStreamHdr = malloc(streamInfo[streamID].OptionalStreamHdrSize);
            char *comments = malloc(streamInfo[streamID].StreamCommentSize);
            char *frameData = malloc(streamInfo[streamID].BytesPerPixel *
                                     streamInfo[streamID].nRowsPerStream *
                                     streamInfo[streamID].nColsPerStream);

            ReadStream(readFileHandle, frameID, streamID, &timeStampInmsecs,
                       optStreamHdr, comments, frameData);

            WriteStream(writeFileHandle, frameID, streamID, timeStampInmsecs,
                        optStreamHdr, comments, frameData);

            free(optStreamHdr);
            free(comments);
            free(frameData);
        }
    }

    Close(readFileHandle);
    Close(writeFileHandle);

    return 0;
}

void printFileHeader(FSFFILEHEADER *fileHeader) {
    printf("\nFileHeader: {");
    printf("\n    HeaderSize: %d", fileHeader->HeaderSize);
    printf("\n    MagicNumber: %d", fileHeader->MagicNumber);
    printf("\n    FileFormatMajorVersion: %d", fileHeader->FileFormatMajorVersion);
    printf("\n    FileFormatMinorVersion: %d", fileHeader->FileFormatMinorVersion);
    printf("\n    FileHeaderSize: %d", fileHeader->FileHeaderSize);
    printf("\n    fsfStreamInfoSize: %d", fileHeader->StreamInfoSize);
    printf("\n    StreamHeaderSize: %d", fileHeader->StreamHeaderSize);
    printf("\n    OptionalFileHdrSize: %d", fileHeader->OptionalFileHdrSize);
    printf("\n    FileCommentSize: %d", fileHeader->FileCommentSize);
    printf("\n    FrameOffsetInfoLoc: %llu", fileHeader->FrameOffsetInfoLoc);
    printf("\n    nFrames: %d", fileHeader->nFrames);
    printf("\n    nStreams: %d", fileHeader->nStreams);
    printf("\n}\n");
}

void printStreamInfo(FSFSTREAMINFO *streamInfo) {
    printf("\nfsfStreamInfo: {");
    printf("\n    SystemID: %d", streamInfo->SystemID);
    printf("\n    StreamType: %d", streamInfo->StreamType);
    printf("\n    ChannelFormat: %d", streamInfo->ChannelFormat);
    printf("\n    BytesPerPixel: %d", streamInfo->BytesPerPixel);
    printf("\n    nRowsPerStream: %d", streamInfo->nRowsPerStream);
    printf("\n    nColsPerStream: %d", streamInfo->nColsPerStream);
    printf("\n    OptionalStreamHdrSize: %d", streamInfo->OptionalStreamHdrSize);
    printf("\n    StreamCommentSize: %d", streamInfo->StreamCommentSize);
    printf("\n    CompressionScheme: %d", streamInfo->CompressionScheme);
    printf("\n}\n");
}

void printStringBuffer(const char *p, size_t s) {
    printf("\n");

    while (s > 0) {
        size_t c = 16;

        if (s < 16) {
            c = s;
        }

        for (size_t i = 0; i < c; i++) {
            char a = p[i];
            printf("%c", isprint(a) ? a : '.');
        }

        for (size_t i = c; i < 16; i++) {
            printf(" ");
        }

        printf(" ");

        for (size_t i = 0; i < c; i++) {
            char a = p[i];
            printf(" %02x", a);
        }

        printf("\n");

        s -= c;
        p += c;
    }
}
