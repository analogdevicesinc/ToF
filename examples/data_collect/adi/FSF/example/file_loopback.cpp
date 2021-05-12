/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
/**
 * @brief   FSF parser loop back example
 *
 * @details This example application uses the FSF parser class to
 *          read and existing FSF file and write the contents to a new file.
 *          The resulting files should be the same.
 */
#include <iostream>
#include <fsf_common.h>

void printFileHeader(aditof::FileHeader &fileHeader);
void printStreamInfo(aditof::StreamInfo &streamInfo);
void printStringBuffer(const char *p, size_t s);

int main(int argc, char *argv[])
{
    if (argc != 3) {
        std::cout << "Please provide path to input and output FSF files.\n\n"
                  << "Example usage:\n"
                  << "   file_loopback.exe <Read_FSF_file> <Write_FSF_file>\n";

        return -1;
    }

    const char *readFile = argv[1];
    const char *writeFile = argv[2];

    // Create platform agnostic FSF parser objects
    aditof::FSF *pFsfRead = new aditof::FSF_Common { aditof::FsfMode::READ };
    aditof::FSF *pFsfWrite = new aditof::FSF_Common { aditof::FsfMode::WRITE };

    pFsfRead->OpenFile(readFile);
    pFsfWrite->CreateFsfFile(writeFile);

    aditof::FileHeader fileHeader {};
    pFsfRead->GetFileHeader(fileHeader);
    pFsfWrite->SetFileHeader(fileHeader);
    printFileHeader(fileHeader);

    for (unsigned int streamIdx = 0; streamIdx < fileHeader.nStreams; streamIdx++) {
        aditof::StreamInfo streamInfo {};
        pFsfRead->GetStreamInfo(streamIdx, streamInfo);
        pFsfWrite->SetStreamInfo(streamIdx, streamInfo);
        printStreamInfo(streamInfo);
    }

    aditof::OptionalFileHeader optFileHeader = "";
    pFsfRead->GetOptionalFileHeader(optFileHeader);
    pFsfWrite->SetOptionalFileHeader(optFileHeader);
    printStringBuffer(optFileHeader.data(), optFileHeader.size());

    aditof::FileComment fileComment  = "";
    pFsfRead->GetFileComment(fileComment);
    pFsfWrite->SetFileComment(fileComment);
    printStringBuffer(fileComment.data(), fileComment.size());

    aditof::Stream stream {};
    for (unsigned int frameIdx = 0; frameIdx < fileHeader.nFrames; frameIdx++) {
        for (unsigned int streamIdx = 0; streamIdx < fileHeader.nStreams; streamIdx++) {
            pFsfRead->GetStream(frameIdx, streamIdx, stream);
            pFsfWrite->SetStream(frameIdx, streamIdx, stream);
        }
    }

    if (pFsfWrite->SaveFile() == aditof::FsfStatus::SUCCESS) {
        std::cout << "\nSuccessfully created " <<  writeFile
                  << " file." << std::endl;
    }

    pFsfRead->CloseFile();
    pFsfWrite->CloseFile();

    return 0;
}

void printFileHeader(aditof::FileHeader &fileHeader)
{
    std::cout << "\nFileHeader: {"
              << "\n    HeaderSize: " << fileHeader.HeaderSize
              << "\n    MagicNumber: " << fileHeader.MagicNumber
              << "\n    FileFormatMajorVersion: " << fileHeader.FileFormatMajorVersion
              << "\n    FileFormatMinorVersion: " << fileHeader.FileFormatMinorVersion
              << "\n    FileHeaderSize: " << fileHeader.FileHeaderSize
              << "\n    StreamInfoSize: " << fileHeader.StreamInfoSize
              << "\n    StreamHeaderSize: " << fileHeader.StreamHeaderSize
              << "\n    OptionalFileHdrSize: " << fileHeader.OptionalFileHdrSize
              << "\n    FileCommentSize: " << fileHeader.FileCommentSize
              << "\n    FrameOffsetInfoLoc: " << fileHeader.FrameOffsetInfoLoc
              << "\n    nFrames: " << fileHeader.nFrames
              << "\n    nStreams: " << fileHeader.nStreams
              << "\n}\n" << std::endl;
}

void printStreamInfo(aditof::StreamInfo &streamInfo)
{
    std::cout << "\nStreamInfo: {"
              << "\n    SystemID: " << streamInfo.SystemID
              << "\n    StreamType: " << streamInfo.StreamType
              << "\n    ChannelFormat: " << streamInfo.ChannelFormat
              << "\n    BytesPerPixel: " << streamInfo.BytesPerPixel
              << "\n    nRowsPerStream: " << streamInfo.nRowsPerStream
              << "\n    nColsPerStream: " << streamInfo.nColsPerStream
              << "\n    OptionalStreamHdrSize: " << streamInfo.OptionalStreamHdrSize
              << "\n    StreamCommentSize: " << streamInfo.StreamCommentSize
              << "\n    CompressionScheme: " << streamInfo.CompressionScheme
              << "\n}\n" << std::endl;
}

void printStringBuffer(const char *p, size_t s)
{
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
