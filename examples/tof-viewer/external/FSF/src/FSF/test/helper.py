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
from shutil import copyfile
import os

sys.path.append('../bindings/python/Debug/')

# raw data file name
readDataFileName = "\\raw_80cm_WhiteTarget_LT.fsf"

def check_status(func, status):
    """

    :param func:
    :param status:
    :return:
    """
    print(func, status)


class FsfData:
    """

    """
    def __init__(self):
        self.fileHeader = fsf.FileHeader()
        self.streamInfo = [fsf.StreamInfo()]
        self.optFileHeader = fsf.OptionalFileHeader()
        self.fileComment = fsf.FileComment()
        self.stream = [[fsf.Stream()]]

    def alloc_stream_info(self):
        self.streamInfo = [fsf.StreamInfo() for k in range(self.fileHeader.nStreams)]

    def alloc_stream(self):
        self.stream = [[fsf.Stream() for k in range(self.fileHeader.nStreams)]
                  for j in range(self.fileHeader.nFrames)]


def read_fsf_file(fsf_data, read_dir, filename):
    """

    :param fsf_data:
    :param read_dir:
    :param filename:
    :return:
    """
    status = 0
    last_function = ""

    try:
        read_file = read_dir + filename

        # FSF read utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)
        status = fsf_read.OpenFile(read_file)
        last_function = "fsf_read.OpenFile"
        assert (status == fsf.Status.SUCCESS)

        # File Header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        last_function = "fsf_read.GetFileHeader"
        assert (status == fsf.Status.SUCCESS)

        # StreamInfo
        fsf_data.streamInfo = [fsf.StreamInfo() for k in range(fsf_data.fileHeader.nStreams)]
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            last_function = "fsf_read.GetStreamInfo"
            assert (status == fsf.Status.SUCCESS)

        # Optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        last_function = "fsf_read.GetOptionalFileHeader"
        assert (status == fsf.Status.SUCCESS)

        # File Comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        last_function = "fsf_read.GetFileComment"
        assert (status == fsf.Status.SUCCESS)

        # FSF Stream
        fsf_data.stream = [[fsf.Stream() for k in range(fsf_data.fileHeader.nStreams)]
                           for j in range(fsf_data.fileHeader.nFrames)]

        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.GetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                last_function = "fsf_read.GetStream"
                assert (status == fsf.Status.SUCCESS)

        status = fsf_read.CloseFile()
        last_function = "fsf_read.CloseFile"
        assert (status == fsf.Status.SUCCESS)

    finally:
        if status != fsf.Status.SUCCESS:
            print("Error: Last function in read_fsf_file was " + last_function)
        return status


def create_dummy_read_file(file_dir, dummy_rd_file="\\dummy.fsf", orig_rd_file="\\80cm_WhiteTarget_LT.fsf"):
    """
    :param file_dir:
    :param orig_rd_file:
    :param dummy_rd_file:
    :return:
    """

    orig_file = file_dir + orig_rd_file
    dummy_file = file_dir + dummy_rd_file

    # Check if original file exists
    if os.path.isfile(orig_file):
        copyfile(orig_file, dummy_file)
        return True
    else:
        print("Error: Can't find original read file")
        return False
