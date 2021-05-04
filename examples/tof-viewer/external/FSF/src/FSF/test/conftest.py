"""
/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
"""
###################################################
#
# Command line options for the pytest
#
import pytest
import time
from helper import *

FSF_TESTFRAMEWORK_VERSION = "0.1"

def pytest_addoption(parser):
    parser.addoption(
            "--all", action="store_false", default=False, help="Run All tests")
    parser.addoption(
            "--writedir", action="store", default=(os.getcwd() + "\\writeFiles"), help="Specify fsf write test files directory")
    parser.addoption(
            "--readdir", action="store", default=(os.getcwd() + "\\readFiles"), help="Specify fsf read test files directory")
    parser.addoption(
            "--rawdata", action="store", default="getFSFFile", help="Specify fsf raw")


@pytest.fixture(scope='session')
def cmdopt_all(pytestconfig):
    return pytestconfig.config.getoption("--all")


@pytest.fixture(scope='session')
def cmdopt_write_dir(pytestconfig):
    return pytestconfig.getoption("--writedir")


@pytest.fixture(scope='session')
def cmdopt_read_dir(pytestconfig):
    return pytestconfig.getoption("--readdir")


@pytest.fixture(scope='session')    # (scope='class')
def cmdopt_raw_data(pytestconfig, cmdopt_read_dir):

    if pytestconfig.getoption("--rawdata") == "getFSFFile":

        fsf_data = FsfData()
        read_fsf_file(fsf_data, cmdopt_read_dir, readDataFileName)

        return fsf_data

    else:
        # do nothing
        print("OTHER RAW DATA SOURCE NOT SUPPORTED")
        assert False

def pytest_report_header(config):
    starttime = time.asctime(time.localtime(time.time()))
    print("Time Start :", starttime)
    print("TEST FRAMEWORK VERSION", FSF_TESTFRAMEWORK_VERSION)


@pytest.yield_fixture(autouse=True, scope='session')
def tf_cleanup():
    yield


#
# This function gets executed before any other methods in the system
# Keep any global initializations here.
#

@pytest.fixture(scope="session", autouse=True)
def tf_init(cmdopt_write_dir, cmdopt_read_dir):

    print("Initalizing the FSF Test Framework")
    print("Removing writeFiles folder")

    save_folder = cmdopt_write_dir + "\\"
    load_folder = cmdopt_read_dir + "\\"

    # Re initialize writeFiles folder
    if os.path.isdir(save_folder):
        files = os.listdir(save_folder)
        for file in files:
            os.remove(save_folder + file)
    else:
        os.mkdir(save_folder, 777)

    # Re initialize readFiles folder
    if os.path.isdir(load_folder):
        files = os.listdir(load_folder)
        for file in files:
            os.remove(load_folder + file)
    else:
        os.mkdir(load_folder, 777)

    cwd = os.getcwd()
    test_data_src = cwd + "\\..\\data\\"

    for filename in os.listdir(test_data_src):
        test_file_src = test_data_src + filename
        test_file_dest = load_folder + filename
        test_file_dest2 = load_folder + "raw_" + filename
        copyfile(test_file_src, test_file_dest)
        copyfile(test_file_src, test_file_dest2)

# What happens when test finishes
def pytest_sessionfinish(session, exitstatus):
    return
