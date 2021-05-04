"""
/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
"""
#################################################################################
#                                                                               #
#  TOF sdk tests                                                                #
#                                                                               #
# test_fsf.py  - Tests specific to SDK are kept here                             #
#                                                                               #
#  API's tested                                     #
#                                                                               #
# Test cases correspond to the test plan                                        #
#################################################################################
import pytest
import sys
import fsf_python as fsf
from helper import *
import os.path as path
from math import ceil


##############################
#
#  FSF parser Create File API test
#
class TestFSFCreateFsfFile:

    @pytest.mark.smoke
    def test_fsf_001_00(self, cmdopt_write_dir):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF and call CreateFsfFile with valid FSF file name
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_00) : "
              "Instantiate FSF and call CreateFsfFile with valid FSF file name\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_001_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_001_01(self, cmdopt_write_dir):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF using write utility and call CreateFsfFile twice
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_01) : "
              "Instantiate FSF using write utility and call CreateFsfFile twice\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_001_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_001_02(self):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF using write utility and call CreateFsfFile with NULL as argument
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_02) : "
              "Instantiate FSF using write utility and call CreateFsfFile with NULL as argument\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename to NULL
        write_file = ""

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert (status is False)

    def test_fsf_001_03(self, cmdopt_write_dir):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_03) : "
              "Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        file_name = "\\test_fsf_001_03.fsf"
        read_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_read.CreateFsfFile(read_file)
        check_status("fsf_read.CreateFsfFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Check if file exists
        status = path.isfile(read_file)
        check_status("path.isfile", status)
        assert (status is False)

    def test_fsf_001_04(self, cmdopt_write_dir):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF using read utility and call CreateFsfFile twice
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_04) : "
              "Instantiate FSF using read utility and call CreateFsfFile twice\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        file_name = "\\test_fsf_001_04.fsf"
        read_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_read.CreateFsfFile(read_file)
        check_status("fsf_read.CreateFsfFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Create FSF file for writing
        status = fsf_read.CreateFsfFile(read_file)
        check_status("fsf_read.CreateFsfFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Check if file exists
        status = path.isfile(read_file)
        check_status("path.isfile", status)
        assert (status is False)

    def test_fsf_001_05(self, cmdopt_write_dir):
        """
        Exercise FSF 'Create File' API
        Instantiate FSF using read utility and call CreateFsfFile with NULL as argument
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_001_05) : "
              "Instantiate FSF using read utility and call CreateFsfFile with NULL as argument\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename to NULL
        read_file = ""

        # Create FSF file for writing
        status = fsf_read.CreateFsfFile(read_file)
        check_status("fsf_read.CreateFsfFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Check if file exists
        status = path.isfile(read_file)
        check_status("path.isfile", status)
        assert (status is False)


##############################
#
#  FSF parser Open File API test
#
class TestFSFOpenFile:

    @pytest.mark.smoke
    def test_fsf_002_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name then OpenFile
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_00) : "
              "Instantiate FSF using read utility and call OpenFile\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_002_01(self, cmdopt_read_dir):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name then OpenFile
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_01) : "
              "Instantiate FSF using read utility and call OpenFile\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName + "non existent"

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.FILE_DOESNOT_EXIST)

    def test_fsf_002_02(self, cmdopt_read_dir):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using read utility and call OpenFile with NULL as argument
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_02) : "
              "Instantiate FSF using read utility and call OpenFile with NULL as argument\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = ""

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.FILE_DOESNOT_EXIST)

    def test_fsf_002_03(self, cmdopt_read_dir):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using write utility and call OpenFile with valid FSF file name
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_03) : "
              "Instantiate FSF using write utility and call OpenFile with valid FSF file name\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        write_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_write.OpenFile(write_file)
        check_status("fsf_write.OpenFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_002_04(self, cmdopt_read_dir):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using write utility and call OpenFile with non-existing FSF file name
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_04) : "
              "Instantiate FSF using write utility and call OpenFile with non-existing FSF file name\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        write_file = cmdopt_read_dir + readDataFileName + "non existent"

        # Create FSF file for writing
        status = fsf_write.OpenFile(write_file)
        check_status("fsf_write.OpenFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_002_05(self):
        """
        Exercise FSF 'Open File' API
        Instantiate FSF using write utility and call OpenFile with NULL as argument
        """
        print("\n===================================================="
              "\nTestFSFCreateFsfFile (test_fsf_002_05) : "
              "Instantiate FSF using write utility and call OpenFile with NULL as argument\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        write_file = ""

        # Create FSF file for writing
        status = fsf_write.OpenFile(write_file)
        check_status("fsf_write.OpenFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

##############################
#
#  FSF parser Close File API test
#
class TestFSFCloseFile:

    @pytest.mark.smoke
    def test_fsf_003_00(self, cmdopt_write_dir):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using write utility and call CreateFsfFile with existing FSF file name then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_00) : "
              "Instantiate FSF using write utility and call CreateFsfFile with existing FSF file name then CloseFile\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_003_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)

        # CloseFile
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is True

    def test_fsf_003_01(self, cmdopt_read_dir):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using read utility and call OpenFile with existing FSF file name then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_00) : "
              "Instantiate FSF using read utility and call OpenFile with existing FSF file name then CloseFile\n   ")

        # Instantiate FSF Read utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for reading
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)

        # Close FSF file
        status = fsf_read.CloseFile()
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_003_02(self):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using write utility, then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_02) : "
              "Instantiate FSF using write utility, then CloseFile\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # CloseFile
        # TODO: This should fail
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_003_03(self):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using read utility,  then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_03) : "
              "Instantiate FSF using read utility,  then CloseFile\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # CloseFile
        # TODO: This should fail
        status = fsf_read.CloseFile()
        check_status("fsf_read.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_003_04(self, cmdopt_write_dir):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using write utilityand call CreateFsfFile with NULL as argument then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_04) : "
              "Instantiate FSF using write utilityand call CreateFsfFile with NULL as argument then CloseFile\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = ""
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.FILE_NOT_CREATED)

        # CloseFile
        # TODO: This should fail
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is False

    def test_fsf_003_05(self, cmdopt_read_dir):
        """
        Exercise FSF 'Close File' API
        Instantiate FSF using read utility and call OpenFile with NULL as argument then CloseFile
        """
        print("\n===================================================="
              "\nTestFSFCloseFile (test_fsf_003_05) : "
              "Instantiate FSF using read utility and call OpenFile with NULL as argument then CloseFile\n   ")

        # Instantiate FSF Read utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = ""

        # Open FSF file for reading
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.FILE_DOESNOT_EXIST)

        # Close FSF file
        # TODO: This should fail
        status = fsf_read.CloseFile()
        assert (status == fsf.Status.SUCCESS)

##############################
#
#  FSF parser SetFileHeader API test
#
class TestFSFSetFileHeader:

    @pytest.mark.smoke
    def test_fsf_004_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileHeader' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetFileHeader with a previously instantiated fileHeader with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileHeader (test_fsf_004_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetFileHeader with a previously instantiated fileHeader with valid values.   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_004_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_004_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileHeader' API
        Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name.
        SetFileHeader with a previously instantiated fileHeader with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileHeader (test_fsf_004_01) : "
              "Instantiate FSF using read utility and call CreateFsfFile with valid FSF file name. "
              "SetFileHeader with a previously instantiated fileHeader with valid values.\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        file_name = "\\test_fsf_004_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_read.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        # assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_004_02(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileHeader' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        SetFileHeader with a previously instantiated fileHeader with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileHeader (test_fsf_004_02) : "
              "Instantiate FSF using read utility and call OpenFile with valid FSF file name. "
              "SetFileHeader with a previously instantiated fileHeader with valid values.\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Create dummy read file
        file_name = "\\test_fsf_004_02.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_write.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_004_03(self, cmdopt_write_dir):
        """
        Exercise FSF 'SetFileHeader' API
        Instantiate using write utility FSF and call CreateFsfFile. SetFileHeader with a fileHeader with no values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileHeader (test_fsf_004_03) : "
              "\nInstantiate using write utility FSF and call CreateFsfFile. "
              "SetFileHeader with a fileHeader with no values.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_004_03.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()

        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status


##############################
#
#  FSF parser GetFileHeader API test
#
class TestFSFGetFileHeader:
    @pytest.mark.smoke
    def test_fsf_005_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetFileHeader' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        GetFileHeader from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileHeader (test_fsf_005_00) : "
              "Instantiate FSF using read utility and call OpenFile with valid FSF file name."
              "GetFileHeader from an FSF file with complete data.\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf_data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

    def test_fsf_005_01(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetFileHeader' API
        Instantiate FSF using read utility and call OpenFile with NULL FSF file name.
        GetFileHeader from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileHeader (test_fsf_005_01) : "
              "Instantiate FSF using read utility and call OpenFile with NULL FSF file name. "
              "GetFileHeader from an FSF file with complete data.\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = ""

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.FILE_DOESNOT_EXIST)

        # Instantiate fsf_data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.FILE_NOT_OPEN)

    def test_fsf_005_02(self, cmdopt_write_dir):
        """
        Exercise FSF 'GetFileHeader' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        GetFileHeader from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileHeader (test_fsf_005_02) : "
              "Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "GetFileHeader from an FSF file .\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_006_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Open FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf_data
        fsf_data = FsfData()

        # Read file header
        status = fsf_write.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)


##############################
#
#  FSF parser SetStreamInfo API test
#
class TestFSFSetStreamInfo:

    @pytest.mark.smoke
    def test_fsf_006_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStreamInfo' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name
        then SetStreamInfo  previously instantiated streamInfo with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetStreamInfo (test_fsf_006_00) : "
              "Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name "
              "then SetStreamInfo previously instantiated streamInfo with valid values.\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_006_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)
            check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is True

    def test_fsf_006_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStreamInfo' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetStreamInfo  previously instantiated streamInfo with no values.
        """
        print("\n===================================================="
              "\nTestFSFSetStreamInfo (test_fsf_006_01) : "
              "Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetStreamInfo  previously instantiated streamInfo with no values.\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_006_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        new_stream_info = [fsf.StreamInfo() for k in range(fsf_data.fileHeader.nStreams)]
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, new_stream_info[streamIdx])
            assert (status == fsf.Status.SUCCESS)
            check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is True

    def test_fsf_006_02(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStreamInfo' API
        Instantiate FSF using read utility and call OpenFile.
        SetStreamInfo  previously instantiated streamInfo with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetStreamInfo (test_fsf_006_00) : "
              "Instantiate FSF using read utility and call OpenFile. "
              "SetStreamInfo  previously instantiated streamInfo with valid values.\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_006_02.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_006_03(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStreamInfo' API
        Instantiate FSF using write utility and call OpenFile.
        SetStreamInfo  previously instantiated streamInfo with valid values WITHOUT setting fileheader.
        """
        print("\n===================================================="
              "\nTestFSFSetStreamInfo (test_fsf_006_03) : "
              "Instantiate FSF using write utility and call OpenFile. "
              "SetStreamInfo  previously instantiated streamInfo with valid values WITHOUT setting fileheader.\n   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_006_03.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set stream info without setting fileheader
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is True


##############################
#
#  FSF parser SetStreamInfo API test
#
class TestFSFGetStreamInfo:

    @pytest.mark.smoke
    def test_fsf_007_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetStreamInfo' API
        Instantiate FSF using read utility and call OpenFile. GetStreamInfo  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetStreamInfo (test_fsf_007_00) : "
              "\nInstantiate FSF using read utility and call OpenFile. "
              "GetStreamInfo  from an FSF file with complete data.  ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)
        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

    def test_fsf_007_01(self, cmdopt_write_dir):
        """
        Exercise FSF 'GetStreamInfo' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        GetStreamInfo  from FSF file with incomplete data.
        """
        print("\n===================================================="
              "\nTestFSFGetStreamInfo (test_fsf_007_01) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "GetStreamInfo  from FSF file with incomplete data.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_007_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()
        assert (fsf_data.fileHeader is not None)

        # Get file header
        status = fsf_write.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

    def test_fsf_007_02(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetStreamInfo' API
        Instantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile.
        Without getting the fileHeader, do GetStreamInfo  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetStreamInfo (test_fsf_007_02) : "
              "\nInstantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile. "
              "Without getting the fileHeader, do GetStreamInfo  from an FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()
        fsf_data.fileHeader.nStreams = 9

        # Read stream info without reading file header
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)
        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)


##############################
#
#  FSF parser SetOptionalFileHeader API test
#
class TestFSFSetOptionalFileHeader:

    @pytest.mark.smoke
    def test_fsf_008_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file.
        SetOptionalFileHeader using  previously instantiated OptionalFileHeader with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file. "
              "SetOptionalFileHeader using  previously instantiated OptionalFileHeader with valid values.   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_008_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_008_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetOptionalFileHeader using  previously instantiated OptionalFileHeader with invalid values. (size greater than the OptionalStreamHdrSize)
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_01) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetOptionalFileHeader using  previously instantiated OptionalFileHeader with invalid values. "
              "(size greater than the OptionalStreamHdrSize)")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_008_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        new_opt_file_hdr = fsf.OptionalFileHeader()
        new_opt_file_hdr.string = fsf_data.optFileHeader.string + fsf_data.optFileHeader.string
        status = fsf_write.SetOptionalFileHeader(new_opt_file_hdr)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.FAILED)

    def test_fsf_008_02(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetOptionalFileHeader using  previously instantiated OptionalFileHeader with invalid values.
        (size less than the OptionalStreamHdrSize)
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_02) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetOptionalFileHeader using  previously instantiated OptionalFileHeader with invalid values. "
              "(size less than the OptionalStreamHdrSize)")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_008_02.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        new_opt_file_hdr = fsf.OptionalFileHeader()
        new_opt_file_hdr.string = ""
        status = fsf_write.SetOptionalFileHeader(new_opt_file_hdr)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.FAILED)

    def test_fsf_008_03(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile.
        Without setting the fileHeader, do SetOptionalFileHeader using  previously
        instantiated OptionalFileHeader with  values.
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_03) : "
              "\nInstantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile. "
              "Without setting the fileHeader, do SetOptionalFileHeader using  previously "
              "instantiated OptionalFileHeader with  values. ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_008_03.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status is True

    def test_fsf_008_04(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile.
        Without setting the streaminfo, do SetOptionalFileHeader using  previously
        instantiated OptionalFileHeader with  values.
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_04) : "
              "\nInstantiate FSF and call CreateFsfFile with valid FSF file name then OpenFile. "
              "Without setting the streaminfo, do SetOptionalFileHeader using  previously "
              "instantiated OptionalFileHeader with  values. ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_008_04.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_008_05(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetOptionalFileHeader' API
        Instantiate FSF using read utility and call OpenFile.
        SetOptionalFileHeader using  previously instantiated OptionalFileHeader with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetOptionalFileHeader (test_fsf_008_05) : "
              "\nInstantiate FSF using read utility and call OpenFile. "
              "SetOptionalFileHeader using  previously instantiated OptionalFileHeader with valid values.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_008_05.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()    # cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        fsf_data.alloc_stream()
        fsf_data.alloc_stream_info()

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_read.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)


##############################
#
#  FSF parser GetOptionalFileHeader API test
#
class TestFSFGetOptionalFileHeader:

    @pytest.mark.smoke
    def test_fsf_009_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetOptionalFileHeader' API
        Instantiate FSF using read utility and call OpenFile.
        GetOptionalFileHeader from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetOptionalFileHeader (test_fsf_009_00) : "
              "\nInstantiate FSF using read utility and call OpenFile. "
              "GetOptionalFileHeader from an FSF file with complete data.   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_009_01(self, cmdopt_write_dir):
        """
        Exercise FSF 'GetOptionalFileHeader' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        GetOptionalFileHeader from FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetOptionalFileHeader (test_fsf_009_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "GetOptionalFileHeader from FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_009_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()

        # Read and write optional file header
        status = fsf_write.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_009_02(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetOptionalFileHeader' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name. GetOptionalFileHeader from FSF file with forced OptionalStreamHdrSize as 0.
        """
        print("\n===================================================="
              "\nTestFSFGetOptionalFileHeader (test_fsf_009_02) : "
              "\nInstantiate FSF using read utility and call OpenFile with valid FSF file name. "
              "GetOptionalFileHeader from FSF file after forced OptionalFileHdrSize as 0.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        print("fsf_data.fileHeader.OptionalFileHdrSize: ", fsf_data.fileHeader.OptionalFileHdrSize)
        fsf_data.fileHeader.OptionalFileHdrSize = 0
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        print(fsf_data.optFileHeader.string)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_009_03(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetOptionalFileHeader' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        Without getting the fileHeader, do GetOptionalFileHeader  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetOptionalFileHeader (test_fsf_009_03) : "
              "\nInstantiate FSF using read utility and call OpenFile with valid FSF file name. "
              "Without getting the fileHeader, do GetOptionalFileHeader  from an FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read stream info without getting file header
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)


##############################
#
#  FSF parser SetFileComment API test
#
class TestFSFSetFileComment:

    @pytest.mark.smoke
    def test_fsf_010_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileComment' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetFileComment using  previously instantiated fileComment with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileComment (test_fsf_010_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetFileComment using  previously instantiated fileComment with valid values.   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_010_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_010_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileComment' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetFileComment using  previously instantiated fileComment with invalid values.
        (size greater than the FileCommentSize)
        """
        print("\n===================================================="
              "\nTestFSFSetFileComment (test_fsf_010_01) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetFileComment using  previously instantiated fileComment with invalid values. "
              "(size greater than the FileCommentSize)   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_010_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        new_file_comment = fsf.FileComment()
        new_file_comment.string = fsf_data.fileComment.string + "random string"
        print(fsf_data.fileComment.string)
        status = fsf_write.SetFileComment(new_file_comment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.FAILED)

    def test_fsf_010_02(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileComment' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetFileComment using  previously instantiated fileComment with invalid values. (
        size less than the FileCommentSize)
        """
        print("\n===================================================="
              "\nTestFSFSetFileComment (test_fsf_010_02) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetFileComment using  previously instantiated fileComment with invalid values. "
              "(size less than the FileCommentSize)")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_010_02.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        file_comment_size = fsf_data.fileHeader.FileCommentSize
        fsf_data.fileHeader.FileCommentSize = 100

        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        fsf_data.fileHeader.FileCommentSize = file_comment_size

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.FAILED)

    def test_fsf_010_03(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileComment' API
        Instantiate FSF using read utility and call OpenFile.
        SetFileComment using  previously instantiated fileComment with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileComment (test_fsf_010_03) : "
              "\nInstantiate FSF using read utility and call OpenFile. "
              "SetFileComment using  previously instantiated fileComment with valid values.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_010_03.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_read.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Write file comment
        status = fsf_read.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_010_04(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetFileComment' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        WITHOUT setting the fileHeader, do SetFileComment using  previously instantiated fileComment with  values.
        """
        print("\n===================================================="
              "\nTestFSFSetFileComment (test_fsf_010_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "WITHOUT setting the fileHeader, do SetFileComment using  previously instantiated "
              "fileComment with  values. ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_010_04.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)


##############################
#
#  FSF parser GetFileComment API test
#
class TestFSFGetFileComment:

    @pytest.mark.smoke
    def test_fsf_011_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetFileComment' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        GetFileComment  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileComment (test_fsf_011_00) : "
              "\nInstantiate FSF using read utility and call OpenFile with valid FSF file name. "
              "GetFileComment  from an FSF file with complete data.   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Read and write file comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_011_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'GetFileComment' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        GetFileComment from FSF file with incomplete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileComment (test_fsf_011_01) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name."
              "GetFileComment from FSF file with complete data.   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_009_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()
        assert (fsf_data.fileHeader is not None)

        # Read and write optional file header
        status = fsf_write.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Read and write file comment
        status = fsf_write.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.INVALID_OPERATION)

    def test_fsf_011_02(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetFileComment' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        GetFileComment from FSF file after forced FileCommentSize as 0.
        """
        print("\n===================================================="
              "\nTestFSFGetFileComment (test_fsf_011_02) : "
              "\nInstantiate FSF using read utility and call OpenFile with valid FSF file name."
              "GetFileComment from FSF file after forced FileCommentSize as 0.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Read and write file comment
        fsf_data.fileHeader.FileCommentSize = 0
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_011_03(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetFileComment' API
        Instantiate FSF using read utility  OpenFile. Without getting the fileHeader,
        do GetFileComment  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetFileComment (test_fsf_011_03) : "
              "\nInstantiate FSF using read utility  OpenFile. Without getting the fileHeader, "
              "do GetFileComment  from an FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf
        fsf_data = FsfData()

        # Read stream info
        fsf_data.streamInfo = [fsf.StreamInfo()] * fsf_data.fileHeader.nStreams
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Read and write file comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)


##############################
#
#  FSF parser SetStream API test
#
class TestFSFSetStream:

    @pytest.mark.smoke
    def test_fsf_012_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStream' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetStream with a previously instantiated stream with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetStream (test_fsf_012_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name then OpenFile. "
              "SetStream with a previously instantiated stream with valid values.   ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_012_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_012_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStream' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetStream using  previously instantiated fileComment with invalid values. (size greater than the nStreams).
        """
        print("\n===================================================="
              "\nTestFSFSetStream (test_fsf_012_01) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetStream using  previously instantiated fileComment with invalid values. "
              "(size greater than the nStreams).")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_012_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Write streams
        new_stream = fsf.Stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                new_stream.optionalStreamHeader = fsf_data.stream[frameIdx][streamIdx].optionalStreamHeader
                new_stream.streamData = fsf_data.stream[frameIdx][streamIdx].streamData + fsf_data.stream[frameIdx][streamIdx].streamData
                status = fsf_write.SetStream(frameIdx, streamIdx, new_stream)
                assert (status == fsf.Status.FAILED)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_012_02(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStream' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        SetStream using  previously instantiated fileComment with invalid values. (size less than the nStreams)
        """
        print("\n===================================================="
              "\nTestFSFSetStream (test_fsf_012_02) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "SetStream using  previously instantiated fileComment with invalid values. (size less than the nStreams)")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_012_02.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Write streams
        new_stream = fsf.Stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, new_stream)
                assert (status == fsf.Status.FAILED)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_012_03(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStream' API
        Instantiate FSF using read utility and call OpenFile.
        SetStream using  previously instantiated fileComment with valid values.
        """
        print("\n===================================================="
              "\nTestFSFSetStream (test_fsf_012_03) : "
              "\nInstantiate FSF using read utility and call OpenFile. "
              "SetStream using  previously instantiated fileComment with valid values.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_012_03.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_read.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Write file comment
        status = fsf_read.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.INVALID_OPERATION)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_012_04(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SetStream' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        WITHOUT setting the fileHeader, do SetStream using  previously instantiated fileComment with  values.
        """
        print("\n===================================================="
              "\nTestFSFSetStream (test_fsf_012_04) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name. "
              "WITHOUT setting the fileHeader, do SetStream using  previously instantiated fileComment with  values. ")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_012_04.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)


##############################
#
#  FSF parser GetStream API test
#
class TestFSFGetStream:

    @pytest.mark.smoke
    def test_fsf_013_00(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetStream' API

        """
        print("\n===================================================="
              "\nTestFSFGetStream (test_fsf_013_00) : "
              "\n   ")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for reading
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.alloc_stream_info()
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Read file comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Read streams
        fsf_data.alloc_stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.GetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_013_01(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'GetStream' API
        Instantiate FSF using write utility and call CreateFsfFile with valid FSF file name.
        GetStream from FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetStream (test_fsf_013_00) : "
              "\nInstantiate FSF using write utility and call CreateFsfFile with valid FSF file name."
              "GetStream from FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_013_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = FsfData()
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.alloc_stream_info()
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_write.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Read and write file comment
        status = fsf_write.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Read streams
        fsf_data.alloc_stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.GetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.INVALID_OPERATION)

        check_status("fsf_read.GetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_013_02(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetStream' API
        Instantiate FSF using read utility and call OpenFile with valid FSF file name.
        GetStream from FSF file after forced StreamSize as 0.
        """
        print("\n===================================================="
              "\nTestFSFGetStream (test_fsf_013_02) : "
              "\nInstantiate FSF using read utility and call OpenFile with valid FSF file name."
              "GetStream from FSF file after forced StreamSize as 0.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for reading
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read file header
        status = fsf_read.GetFileHeader(fsf_data.fileHeader)
        check_status("fsf_read.GetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)
        assert (fsf_data.fileHeader is not None)

        # Read stream info
        fsf_data.alloc_stream_info()
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            fsf_data.streamInfo[streamIdx].nColsPerStream = 0
            fsf_data.streamInfo[streamIdx].nRowsPerStream = 0
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Read file comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Read streams
        fsf_data.alloc_stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.GetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_read.GetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

    def test_fsf_013_03(self, cmdopt_read_dir):
        """
        Exercise FSF 'GetStream' API
        Instantiate FSF using read utility  OpenFile.
        Without getting the fileHeader, do GetFileComment  from an FSF file with complete data.
        """
        print("\n===================================================="
              "\nTestFSFGetStream (test_fsf_013_03) : "
              "\nInstantiate FSF using read utility  OpenFile. "
              "Without getting the fileHeader, do GetFileComment  from an FSF file with complete data.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        read_file = cmdopt_read_dir + readDataFileName

        # Open FSF file for reading
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Instantiate fsf data
        fsf_data = FsfData()

        # Read stream info
        fsf_data.alloc_stream_info()
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.GetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_read.GetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + "streams.", status)

        # Read and write optional file header
        status = fsf_read.GetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_read.GetOptionalFileHeader", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Read file comment
        status = fsf_read.GetFileComment(fsf_data.fileComment)
        check_status("fsf_read.GetFileComment", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Read streams
        fsf_data.alloc_stream()
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.GetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.FILE_HEADER_ERROR)

        check_status("fsf_read.GetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)


##############################
#
#  FSF parser SaveFile API test
#
class TestFSFSaveFile:

    @pytest.mark.smoke
    def test_fsf_014_00(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename, populate FSF data then SaveFile
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_00) : "
              "\nInstantiate FSF using write utility  with valid FSF filename, populate FSF data then SaveFile")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_014_01(self, cmdopt_write_dir):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating data.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_01) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile without populating data.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_01.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.FILE_HEADER_ERROR)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

    def test_fsf_014_02(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating streamInfo.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_02) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then "
              "SaveFile without populating streamInfo.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_02.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.FAILED)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_03(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating optFileHeader.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_03) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile "
              "without populating optFileHeader.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_03.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_04(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating fileComment.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_04) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile without populating fileComment.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_00.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_05(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating stream.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_05) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile without populating stream.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_05.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.SUCCESS)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_06(self, cmdopt_read_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using read utility  with valid FSF filename, populate data then SaveFile.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_06) : "
              "\nInstantiate FSF using read utility  with valid FSF filename, populate data then SaveFile.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_014_06.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_read.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_read.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.INVALID_OPERATION)
        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_read.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Write file comment
        status = fsf_read.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_read.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.INVALID_OPERATION)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_read.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Close file
        status = fsf_read.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_07(self, cmdopt_read_dir):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using read utility  with valid FSF filename then SaveFile without populating data.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_06) : "
              "\nInstantiate FSF using read utility  with valid FSF filename then SaveFile without populating data.")

        # Instantiate FSF Write utility
        fsf_read = fsf.FSF_Common(fsf.Mode.READ)

        # Set FSF filename
        # Create dummy read file
        file_name = "\\test_fsf_014_07.fsf"
        status = create_dummy_read_file(cmdopt_read_dir, file_name)
        assert status is True

        # Set FSF filename
        read_file = cmdopt_read_dir + file_name

        # Create FSF file for writing
        status = fsf_read.OpenFile(read_file)
        check_status("fsf_read.OpenFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Save file
        status = fsf_read.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.INVALID_OPERATION)

        # Close file
        status = fsf_read.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_08(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating half the frame.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_08) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile without "
              "populating half the frame.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_08.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)

        # Write streams
        for frameIdx in range(int(ceil(fsf_data.fileHeader.nFrames/2))):
            for streamIdx in range(fsf_data.fileHeader.nStreams):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)

    def test_fsf_014_09(self, cmdopt_write_dir, cmdopt_raw_data):
        """
        Exercise FSF 'SaveFile' API
        Instantiate FSF using write utility  with valid FSF filename then SaveFile without populating half the stream.
        """
        print("\n===================================================="
              "\nTestFSFSaveFile (test_fsf_014_09) : "
              "\nInstantiate FSF using write utility  with valid FSF filename then SaveFile without "
              "populating half the stream.")

        # Instantiate FSF Write utility
        fsf_write = fsf.FSF_Common(fsf.Mode.WRITE)

        # Set FSF filename
        file_name = "\\test_fsf_014_09.fsf"
        write_file = cmdopt_write_dir + file_name

        # Create FSF file for writing
        status = fsf_write.CreateFsfFile(write_file)
        check_status("fsf_write.CreateFsfFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Get data to use
        fsf_data = cmdopt_raw_data
        assert (fsf_data.fileHeader is not None)

        # Set file header
        status = fsf_write.SetFileHeader(fsf_data.fileHeader)
        check_status("fsf_write.SetFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Set stream info
        for streamIdx in range(fsf_data.fileHeader.nStreams):
            status = fsf_write.SetStreamInfo(streamIdx, fsf_data.streamInfo[streamIdx])
            assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStreamInfo: " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Write optional file header
        status = fsf_write.SetOptionalFileHeader(fsf_data.optFileHeader)
        check_status("fsf_write.SetOptionalFileHeader", status)
        assert (status == fsf.Status.SUCCESS)

        # Write file comment
        status = fsf_write.SetFileComment(fsf_data.fileComment)
        check_status("fsf_write.SetFileComment", status)

        # Write streams
        for frameIdx in range(fsf_data.fileHeader.nFrames):
            for streamIdx in range(int(ceil(fsf_data.fileHeader.nStreams/2))):
                status = fsf_write.SetStream(frameIdx, streamIdx, fsf_data.stream[frameIdx][streamIdx])
                assert (status == fsf.Status.SUCCESS)

        check_status("fsf_write.SetStream: " + str(fsf_data.fileHeader.nFrames) +
                     " frames, " + str(fsf_data.fileHeader.nStreams) + " streams.", status)

        # Save file
        status = fsf_write.SaveFile()
        check_status("fsf_write.SaveFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Close file
        status = fsf_write.CloseFile()
        check_status("fsf_write.CloseFile", status)
        assert (status == fsf.Status.SUCCESS)

        # Check if file exists
        status = path.isfile(write_file)
        check_status("path.isfile", status)
        assert status

        # Check if file can be read
        fsf_read_data = FsfData()
        status = read_fsf_file(fsf_data=fsf_read_data, read_dir=cmdopt_write_dir, filename=file_name)
        check_status("read_fsf_file", status)
        assert (status == fsf.Status.SUCCESS)


##############################
#
#  FSF parser Stress tests
#
class TestFSFStressTests:
    @pytest.mark.smoke
    def test_fsf_015_00(self, cmdopt_write_dir):
        """
        Stress test FSF APIs

        """
        print("\n===================================================="
              "\nTestFSFStressTests (test_fsf_015_00) : "
              "\n   ")
