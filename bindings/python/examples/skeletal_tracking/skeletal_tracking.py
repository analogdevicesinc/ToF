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
import numpy as np
import cv2 as cv
import argparse
from enum import Enum
import sys
import mediapipe as mp

ip = "10.43.0.1" # Set to "10.43.0.1" if networking is used.
config = "config/config_adsd3500_adsd3100.json"
mode = "sr-qnative"

inWidth = 300
inHeight = 300
WHRatio = inWidth / float(inHeight)
inScaleFactor = 0.007843
meanVal = 127.5
thr = 0.2
WINDOW_NAME = "Display Objects"
WINDOW_NAME_DEPTH = "Display Objects Depth"


# Initialize Mediapipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


if __name__ == "__main__":

    system = tof.System()

    cameras = []
    status = system.getCameraList(cameras, "ip:"+ip)
    if not status:
        print("system.getCameraList(): ", status)

    camera1 = cameras[0]
        
    status = camera1.setControl("initialization_config", config)
    print("camera1.setControl()", status)

    status = camera1.initialize()
    if not status:
        print("camera1.initialize() failed with status: ", status)

    modes = []
    status = camera1.getAvailableModes(modes)
    if not status:
        print("system.getAvailableModes() failed with status: ", status)

    status = camera1.setMode(modes[ModesEnum.MODE_NEAR.value])
    if not status:
        print("camera1.setMode() failed with status: ", status)

    types = []
    status = camera1.getAvailableFrameTypes(types)
    if not status:
        print("system.getAvailableFrameTypes() failed with status: ", status)
        
    status = camera1.setFrameType(mode)
    if not status:
        print("camera1.setFrameType() failed with status:", status)
    
    status = camera1.start()
    if not status:
        print("camera1.start() failed with status:", status)
   
    camDetails = tof.CameraDetails()
    status = camera1.getDetails(camDetails)
    if not status:
        print("system.getDetails() failed with status: ", status)

    # Enable noise reduction for better results
    smallSignalThreshold = 100
    camera1.setControl("noise_reduction_threshold", str(smallSignalThreshold))

    camera_range = 5000
    bitCount = 9
    frame = tof.Frame()

    max_value_of_IR_pixel = 2 ** bitCount - 1
    distance_scale_ir = 255.0 / max_value_of_IR_pixel
    distance_scale = 255.0 / camera_range

    while True:
            
        # Capture frame-by-frame
        status = camera1.requestFrame(frame)
        if not status:
            print("camera1.requestFrame() failed with status: ", status)

        ir_map = np.array(frame.getData("ir"), dtype="uint16", copy=False)

        # Creation of the IR image
        ir_map = distance_scale_ir * ir_map
        ir_map = np.uint8(ir_map)
        ir_map = cv.flip(ir_map, 1)
        ir_map_rgb = cv.cvtColor(ir_map, cv.COLOR_GRAY2RGB)  
        ir_map_bgr = cv.cvtColor(ir_map_rgb, cv.COLOR_RGB2BGR)
        
        # Process the frame with Mediapipe Pose and Hands
        results_pose = pose.process(ir_map_rgb)
        results_hands = hands.process(ir_map_rgb)
        
        # Check if any pose is detected
        if results_pose.pose_landmarks:
        	# Draw connections between landmarks
        	mp.solutions.drawing_utils.draw_landmarks(ir_map_bgr, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        
        # Check if any hand is detected
        if results_hands.multi_hand_landmarks:
        	for hand_landmarks in results_hands.multi_hand_landmarks:
        		for landmark in hand_landmarks.landmark:
        			x, y, z = int(landmark.x * ir_map_bgr.shape[1]), int(landmark.y * ir_map_bgr.shape[0]), landmark.z
        			cv.circle(ir_map_bgr, (x, y), 8, (0, 255, 0), -1)

        cv.namedWindow(WINDOW_NAME, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME, ir_map_bgr)

        if cv.waitKey(1) >= 0:
            break
            
    camera1.stop()
