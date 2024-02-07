#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import aditofpython as tof
import pytest
import os.path
import numpy as np
import time 

@pytest.fixture()   
def system():
    system_tof = tof.System()
    return system_tof
    
@pytest.fixture(autouse=True)
def delay_between_tests():
    time.sleep(2) # sleep for 1 second
    
@pytest.fixture(params=[0,1,2,3,4,5,6])
def available_modes_ini(request):
    return request.param
    
#test get camera list
def test_get_camera_list_invalid_indexerror(system):
    with pytest.raises(IndexError):
        # Create an empty list to store the cameras
        cameras = []
        # Call the getCameraList method with some input
        status = system.getCameraList(cameras, "ip:sample")
        print(status)

def test_get_camera_list_invalid_typeerror(system):
    with pytest.raises(TypeError):
        # Create an empty list to store the cameras
        cameras = ()
        # Call the getCameraList method with some input
        status = system.getCameraList(cameras, "10.42.0.1")
        print(status)
        
#test get_camera_list when no hardware is connected(should return unreachable)       
def test_get_camera_list_unreachable(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.2")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Unreachable
    # Assert that the cameras list is not empty
    assert not cameras
     
def test_camera_initialize_error(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    # get the stored camera on index zero
    camera1 = cameras[0]
    
    status = camera1.initialize('sample')
    # Assert that the status is OK
    assert status == tof.Status.GenericError  

def test_get_camera_list_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)    
 
def test_camera_initialize_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    # Assert that the status is OK
    assert status == tof.Status.Ok

#START       
def test_camera_start(system):

    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    status = camera1.start()
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
#STOP   
def test_camera_stop(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    status = camera1.start()
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    status = camera1.stop()
    
    # Assert that the status is OK
    assert status == tof.Status.Ok

@pytest.mark.skip(reason="test after")   
def test_camera_setMode():
    assert True
    
def test_camera_getAvailableModes_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    assert isinstance(frame_types, list), "Should return list"

def test_camera_setFrameType_ok(system):

    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    for frame_type in frame_types:
        print(frame_type)
        status = camera1.setFrameType(frame_type)
        assert status == tof.Status.Ok
    
def test_camera_getAvailableFrameTypes_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    assert isinstance(frame_types, list)

@pytest.mark.skip(reason="test after")    
def test_camera_getFrameTypeNameFromId_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    print(frame_types)

    index = 0
    for frame_type in frame_types:
        mode_name = 'none'
        status = camera1.getFrameTypeNameFromId(index, str(mode_name))
        print(status[1])
        assert status[0] == tof.Status.Ok and status[1] == frame_type
        index = index + 1
      
def test_camera_requestFrame_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.start()
    assert status == tof.Status.Ok
    
    frame = tof.Frame()

    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok
    
    status = camera1.stop()
    assert status == tof.Status.Ok

def test_camera_getDetails_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    camera_details = tof.CameraDetails()
    status = camera1.getDetails(camera_details)
    assert status == tof.Status.Ok
    
    assert isinstance(camera_details, tof.CameraDetails)
    assert isinstance(camera_details.cameraId, str)
    assert isinstance(camera_details.mode, str)
    assert isinstance(camera_details.frameType, tof.FrameDetails)
    assert isinstance(camera_details.connection, tof.ConnectionType)
    assert isinstance(camera_details.intrinsics, tof.IntrinsicParameters)
    assert isinstance(camera_details.maxDepth, int)
    assert isinstance(camera_details.minDepth, int)
    assert isinstance(camera_details.bitCount, int)
    assert isinstance(camera_details.uBootVersion, str)
    assert isinstance(camera_details.kernelVersion, str)    
    assert isinstance(camera_details.sdCardImageVersion, str)  
    assert isinstance(camera_details.serialNumber, str)
    
@pytest.mark.skip(reason="test after")       
def test_camera_getAvailableControls_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    control_list = []
    status = camera1.getAvailableControls(control_list)
    assert status == tof.Status.Ok
    print(control_list)
    assert isinstance(control_list, list), "Should return list"
    
@pytest.mark.skip(reason="test after")      
def test_camera_getControl_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    camera_details = tof.CameraDetails()
    status = camera1.getDetails(camera_details)
    assert status == tof.Status.Ok
    
    control = 'none'
    status = camera1.getControl(str(control))
    assert status == tof.Status.Ok
    print(control)
    assert isinstance(control,str), "Should return list"
    
@pytest.mark.skip(reason="test after")      
def test_camera_setControl_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    control = 'none'
    status = camera1.getControl(str(control))
    assert status == tof.Status.Ok
    print(control)
    assert isinstance(control,str), "Should return list"
    
def test_camera_getSensor_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    sensor = camera1.getSensor()
    print(sensor)
    assert sensor is not None 
    
def test_camera_enableXYZframe_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.enableXYZframe(True)
    assert status == tof.Status.Ok
    
def test_camera_saveModuleCCB_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.saveModuleCCB('./')
    assert status == tof.Status.Ok
    assert os.path.isfile('./temp_ccb.ccb')


@pytest.mark.skip(reason="not yet fully functional")
def test_camera_saveModuleCFG_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.saveModuleCFG('./')
    assert status == tof.Status.Ok
    #assert os.path.isfile('./temp_ccb.ccb')
    
@pytest.mark.skip(reason="not yet fully functional")
def test_camera_saveModuleCFG_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.saveModuleCFG('./')
    assert status == tof.Status.Ok
    
@pytest.mark.skip(reason="removed?")
def test_camera_enableDepthCompute_ok():
    pass

#@pytest.mark.skip(reason="Not supported?")
def test_camera_readSerialNumber_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]
    
    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    serial_number = 'none'
    status = camera1.readSerialNumber(serial_number,True)
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], str)
    

#FRAME 
def test_frame_getDataDetails_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    for frame_type in frame_types:
        print(frame_type)
        status = camera1.setFrameType(frame_type)
        assert status == tof.Status.Ok
 
        status = camera1.start()
        assert status == tof.Status.Ok
    
        frame = tof.Frame()    
        
        status = camera1.requestFrame(frame)
        assert status == tof.Status.Ok  
        
        frameDataDetails = tof.FrameDataDetails()
        frameModes = ['ab', 'depth', 'conf', 'metadata', 'xyz']
        
        for frameMode in frameModes:
            if (frame_type == 'pcm-native') and (frameMode != 'ab'):
                break

            status = frame.getDataDetails(frameMode, frameDataDetails)
            assert status == tof.Status.Ok
            
            assert isinstance(frameDataDetails.width, int)
            assert isinstance(frameDataDetails.height, int)
            assert isinstance(frameDataDetails.type, str)
            assert isinstance(frameDataDetails.subelementSize, int)
            assert isinstance(frameDataDetails.subelementsPerElement, int)
            assert isinstance(frameDataDetails.bytesCount, int)
            time.sleep(2)
        status = camera1.stop()
        assert status == tof.Status.Ok

'''        
def test_frame_getDetails_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    #for frame_type in frame_types:
    #print('sr-native')
    #status = camera1.setFrameType(frame_type)
    status = camera1.setFrameType('sr-native')
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
    
    frameDataDetails = tof.FrameDataDetails()
    frameDetails = tof.FrameDetails()

    status = frame.getDetails(frameDetails)
    assert status == tof.Status.Ok
    
    assert isinstance(frameDetails.type, str)
    assert isinstance(frameDetails.dataDetails,list)
    assert isinstance(frameDetails.cameraMode, str)
    assert isinstance(frameDetails.width, int)
    assert isinstance(frameDetails.height, int)
    assert isinstance(frameDetails.totalCaptures, int)
    assert isinstance(frameDetails.passiveIRCaptured, bool)    
    

    
    status = camera1.stop()
    assert status == tof.Status.Ok
    
def test_frame_getDetails_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    #for frame_type in frame_types:
    #print('sr-native')
    #status = camera1.setFrameType(frame_type)
    status = camera1.setFrameType('sr-native')
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
    
    frameDataDetails = tof.FrameDataDetails()
    frameDetails = tof.FrameDetails()

    status = frame.getDetails(frameDetails)
    assert status == tof.Status.Ok
    
    assert isinstance(frameDetails.type, str)
    assert isinstance(frameDetails.dataDetails,list)
    assert isinstance(frameDetails.cameraMode, str)
    assert isinstance(frameDetails.width, int)
    assert isinstance(frameDetails.height, int)
    assert isinstance(frameDetails.totalCaptures, int)
    assert isinstance(frameDetails.passiveIRCaptured, bool)    

    status = camera1.stop()
    assert status == tof.Status.Ok
  

def test_frame_getMetadataStruct_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    for frame_type in frame_types:
        print(frame_type)
        status = camera1.setFrameType(frame_type)
        assert status == tof.Status.Ok
 
        status = camera1.start()
        assert status == tof.Status.Ok
    
        frame = tof.Frame()    
        
        status = camera1.requestFrame(frame)
        assert status == tof.Status.Ok  
        
        frameDataDetails = tof.FrameDataDetails()
        frameModes = ['ab', 'depth', 'conf', 'metadata', 'xyz']
        
        for frameMode in frameModes:
            if (frame_type == 'pcm-native'):
                break

            status, metadata = frame.getMetadataStruct()
            assert status == tof.Status.Ok

            assert isinstance(metadata.width, int)
            assert isinstance(metadata.height, int)
            assert isinstance(metadata.outputConfiguration, int)
            assert isinstance(metadata.bitsInDepth, int)
            assert isinstance(metadata.bitsInConfidence, int)
            assert isinstance(metadata.invalidPhaseValue, int)
            assert isinstance(metadata.frequencyIndex, int)
            assert isinstance(metadata.abFrequencyIndex, int)
            assert isinstance(metadata.frameNumber, int)
            assert isinstance(metadata.imagerMode, int)
            assert isinstance(metadata.numberOfPhases, int)
            assert isinstance(metadata. numberOfFrequencies, int)
            assert isinstance(metadata.elapsedTimeFractionalValue, int)
            assert isinstance(metadata.elapsedTimeSecondsValue, int)
            assert isinstance(metadata.sensorTemperature, int)
            assert isinstance(metadata.laserTemperature, int)
            
        status = camera1.stop()
        assert status == tof.Status.Ok
        
def test_frame_getAvailableAttributes_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    #for frame_type in frame_types:
        #print(frame_type)
        #status = camera1.setFrameType(frame_type)
    print('sr-native')
    status = camera1.setFrameType('sr-native')
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
          
    attributes = []
    status = frame.getAvailableAttributes(attributes)
    assert status == tof.Status.Ok
    assert isinstance(attributes, list)
    print(attributes)
    status = camera1.stop()
    assert status == tof.Status.Ok

#@pytest.mark.skip(reason="why no return value?")        
def test_frame_getAttribute_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")

    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    #for frame_type in frame_types:
    status = camera1.setFrameType('lr-mixed')
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
    
    value = None     
    attributes = []
    status = frame.getAvailableAttributes(attributes)
    assert status == tof.Status.Ok
    assert isinstance(attributes, list)
    
    status = frame.setAttribute('heeight','100')
    assert status == tof.Status.Ok
    print(status)
    status = frame.getAttribute('height',str(value))
    assert status == tof.Status.Ok
    print(value)
    print(status)
        
    status = camera1.stop()
    assert status == tof.Status.Ok
        
 '''
#ADSD3500
def test_camera_GetEnableMetadatainAB_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    for frame_type in frame_types:
        print(frame_type)
        status = camera1.setFrameType(frame_type)
        assert status == tof.Status.Ok
    
        status = camera1.adsd3500GetEnableMetadatainAB();
        assert status[0] == tof.Status.Ok
        if frame_type == 'pcm-native':
            assert status[1] == 0
        else:
            assert status[1] == 1

def test_camera_SetEnableMetadatainAB_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok

    #status = camera1.setFrameType(frame_type)
    #assert status == tof.Status.Ok
    enable = 0
    camera1.adsd3500SetEnableMetadatainAB(enable);
    status = camera1.adsd3500GetEnableMetadatainAB();
    assert status[0] == tof.Status.Ok
    assert status[1] == 0
    
    enable = 1
    camera1.adsd3500SetEnableMetadatainAB(enable);
    status = camera1.adsd3500GetEnableMetadatainAB();
    assert status[0] == tof.Status.Ok
    assert status[1] == 1
    
    enable = 25
    camera1.adsd3500SetEnableMetadatainAB(enable);
    status = camera1.adsd3500GetEnableMetadatainAB();
    assert status[0] == tof.Status.Ok
    assert status[1] == 1
    
def test_camera_adsd3500GetStatus_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    chip_status = 0
    imager_status = 0
    status = camera1.adsd3500GetStatus(chip_status, imager_status)
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert isinstance(status[2], int)
    
def test_camera_adsd3500GetFirmwareVersion_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    
    # Assert that the status is OK
    assert status == tof.Status.Ok
    
    firmware_vr = ''
    hash_vr = ''
    status = camera1.adsd3500GetFirmwareVersion(firmware_vr, hash_vr)
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], str)
    assert isinstance(status[2], str)  

def test_camera_adsd3500GetSensorTemperature_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    frame_type = 'sr-native'
    status = camera1.setFrameType(frame_type)
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
    
    frameDataDetails = tof.FrameDataDetails()
            
    status = camera1.adsd3500GetSensorTemperature()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
        
    status = camera1.stop()
    assert status == tof.Status.Ok
    
def test_camera_adsd3500GetLaserTemperature_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    frame_types = []
    status = camera1.getAvailableFrameTypes(frame_types)
    assert status == tof.Status.Ok
    
    frame_type = 'sr-native'
    status = camera1.setFrameType(frame_type)
    assert status == tof.Status.Ok

    status = camera1.start()
    assert status == tof.Status.Ok

    frame = tof.Frame()    
    
    status = camera1.requestFrame(frame)
    assert status == tof.Status.Ok  
    
    frameDataDetails = tof.FrameDataDetails()
            
    status = camera1.adsd3500GetLaserTemperature()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
        
    status = camera1.stop()
    assert status == tof.Status.Ok
    
def test_camera_adsd3500GetRadialThresholdMax_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetRadialThresholdMax()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    
def test_camera_adsd3500GetRadialThresholdMin_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetRadialThresholdMin()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)

def test_camera_adsd3500SetRadialThresholdMin_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = 30
    status = camera1.adsd3500SetRadialThresholdMin(test_value)
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetRadialThresholdMin()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert status[1] == test_value

def test_camera_adsd3500SetRadialThresholdMax_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = 4200
    status = camera1.adsd3500SetRadialThresholdMax(test_value)
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetRadialThresholdMax()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert status[1] == test_value
    print(status)
    
def test_camera_adsd3500GetJBLFfilterSize_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetJBLFfilterSize()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)

def test_camera_adsd3500SetJBLFfilterSize_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = 7
    status = camera1.adsd3500SetJBLFfilterSize(test_value)
    assert status == tof.Status.Ok
    
    
    status = camera1.adsd3500GetJBLFfilterSize()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert status[1] == test_value
    
def test_camera_adsd3500GetJBLFfilterEnableState_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetJBLFfilterEnableState()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], bool)
    
def test_camera_adsd3500SetJBLFfilterEnableState_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = True
    status = camera1.adsd3500SetJBLFfilterEnableState(test_value)
    assert status == tof.Status.Ok
    
    
    status = camera1.adsd3500GetJBLFfilterEnableState()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], bool)
    assert status[1] == test_value
    
def test_camera_adsd3500GetConfidenceThreshold_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetConfidenceThreshold()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    print(status)
def test_camera_adsd3500SetConfidenceThreshold_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = 25
    status = camera1.adsd3500SetConfidenceThreshold(test_value)
    assert status == tof.Status.Ok 
    
    status = camera1.adsd3500GetConfidenceThreshold()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert status[1] == test_value
    
def test_camera_adsd3500GetABinvalidationThreshold_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    status = camera1.adsd3500GetABinvalidationThreshold()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)

def test_camera_adsd3500SetABinvalidationThreshold_ok(system):
    # Create an empty list to store the cameras
    cameras = []
    # Call the getCameraList method with some input
    status = system.getCameraList(cameras, "ip:10.42.0.1")
    print(status)
    # Assert that the status is OK
    assert status == tof.Status.Ok
    # Assert that the cameras list is not empty
    assert cameras
    # Assert that the cameras list contains aditof::Camera objects
    for camera in cameras:
        assert isinstance(camera, tof.Camera)
    
    camera1 = cameras[0]

    status = camera1.initialize('config/config_adsd3500_adsd3100.json')
    assert status == tof.Status.Ok
    
    test_value = 25
    status = camera1.adsd3500SetABinvalidationThreshold(test_value)
    assert status == tof.Status.Ok 
    
    status = camera1.adsd3500GetABinvalidationThreshold()
    assert status[0] == tof.Status.Ok
    assert isinstance(status[1], int)
    assert status[1] == test_value
    print(status)
