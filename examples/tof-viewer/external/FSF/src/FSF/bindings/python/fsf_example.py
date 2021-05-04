"""
/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
"""
import sys
import fsf_python as fsf


def checkStatus(func, status):
    if status != fsf.Status.SUCCESS:
        print("{:s}:".format(func.__name__), status)


def main():
    print("\n********** FSF Utility Example with Python Bindings **********\n")

    if len(sys.argv) != 3:
        print("Please provide read an write FSF files.\n\n" \
              "Example usage:\n" \
              "   python3 fsf_example.py <FSF_Read_file> <FSF_Write_file>\n")
        exit(-1)

    # Get file names from args
    readFile = sys.argv[1]
    writeFile = sys.argv[2]

    # FSF read and write utilities
    fsfRead = fsf.FSF_Common(fsf.Mode.READ)
    fsfWrite = fsf.FSF_Common(fsf.Mode.WRITE)

    # Open FSF file for reading
    status = fsfRead.OpenFile(readFile)
    checkStatus(fsfRead.OpenFile, status)

    # Create FSF file for writing
    status = fsfWrite.CreateFsfFile(writeFile)
    checkStatus(fsfWrite.CreateFsfFile, status)

    # Read and write file header
    fileHeader = fsf.FileHeader()

    status = fsfRead.GetFileHeader(fileHeader)
    checkStatus(fsfRead.GetFileHeader, status)

    status = fsfWrite.SetFileHeader(fileHeader)
    checkStatus(fsfWrite.SetFileHeader, status)

    # Read and write stream info
    for streamIdx in range(fileHeader.nStreams):
        streamInfo = fsf.StreamInfo()

        status = fsfRead.GetStreamInfo(streamIdx, streamInfo)
        checkStatus(fsfRead.GetStreamInfo, status)

        status = fsfWrite.SetStreamInfo(streamIdx, streamInfo)
        checkStatus(fsfWrite.SetStreamInfo, status)

    # Read and write optional file header
    optFileHeader = fsf.OptionalFileHeader()

    status = fsfRead.GetOptionalFileHeader(optFileHeader)
    checkStatus(fsfRead.GetOptionalFileHeader, status)

    status = fsfWrite.SetOptionalFileHeader(optFileHeader)
    checkStatus(fsfWrite.SetOptionalFileHeader, status)

    # Read and write file comment
    fileComment = fsf.FileComment()

    status = fsfRead.GetFileComment(fileComment)
    checkStatus(fsfRead.GetFileComment, status)

    status = fsfWrite.SetFileComment(fileComment)
    checkStatus(fsfWrite.SetFileComment, status)

    # Read and write streams
    for frameIdx in range(fileHeader.nFrames):
        for streamIdx in range(fileHeader.nStreams):
            stream = fsf.Stream()

            status = fsfRead.GetStream(frameIdx, streamIdx, stream)
            checkStatus(fsfRead.GetStream, status)

            status = fsfWrite.SetStream(frameIdx, streamIdx, stream)
            checkStatus(fsfWrite.SetStream, status)

    status = fsfWrite.SaveFile()
    checkStatus(fsfWrite.SaveFile, status)

    if status == fsf.Status.SUCCESS:
        print("\nSuccessfully created", writeFile, "file.")

    fsfRead.CloseFile()
    fsfWrite.CloseFile()


if __name__ == '__main__':
    main()
