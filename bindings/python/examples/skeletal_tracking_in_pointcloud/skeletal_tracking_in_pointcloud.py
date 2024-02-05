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

# Import necessary libraries
import aditofpython as tof
import numpy as np
import cv2 as cv
import argparse
from enum import Enum
import sys
import mediapipe as mp
import open3d as o3d
import time
import numba as nb
from numba import cuda
import torch

frame_count = 0
start_time = time.time()

# Initialize Parameters
ip = "10.43.0.1" # Set to "10.43.0.1" if networking is used.
config = "config/config_adsd3500_adsd3100.json"
mode = "sr-qnative"
image_width = 512
image_height = 512
apply_scaling = 1 # Applies Scaling
track_pose = 1 # Applies Pose Tracking
track_hand = 0 # Applies Hand tracking
reduction_factor = 2 # Downsamples the image by a the factor

inWidth = 300
inHeight = 300
WHRatio = inWidth / float(inHeight)
inScaleFactor = 0.007843
meanVal = 127.5
thr = 0.2

class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2

def ApplyLogScaling(input_image):
    max_value = np.max(input_image)
    intensity_range = max_value - np.min(input_image)
    a = 255.0 / intensity_range
    c = 255.0 / np.log10(1 + max_value)

    scaled_image = a * input_image
    scaled_image[scaled_image >= 255.0] = 255.0

    log_scaled_image = c * np.log10(scaled_image + 1)

    return log_scaled_image.astype(np.uint8)

def TrackSkeletonPose(ab_image):
    mp_drawing = mp.solutions.drawing_utils
    results_pose = pose.process(ab_image)

    if results_pose.pose_landmarks:
        mp_drawing.draw_landmarks(
            ab_image, 
            results_pose.pose_landmarks, 
            connections=mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1),
            connection_drawing_spec=mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=1, circle_radius=1)
        )

    return ab_image

def TrackHandMovement(ab_image):
    results_hands = hands.process(ab_image)
    if results_hands.multi_hand_landmarks:
        for hand_landmarks in results_hands.multi_hand_landmarks:
            for landmark in hand_landmarks.landmark:
                x, y, z = int(landmark.x * ab_image.shape[1]), int(landmark.y * ab_image.shape[0]), landmark.z
                cv.circle(ab_image, (x, y), 8, (0, 255, 0), -1)
    return ab_image

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

    # Initialize Mediapipe
    if track_pose:
        mp_pose = mp.solutions.pose
        pose = mp_pose.Pose()

    if track_hand:
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands()

    # Create a visualizer
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()

    point_cloud = o3d.geometry.PointCloud()
    first_time_renderer = 1

    # Modify the point cloud size
    point_size = 2.5
    opt = visualizer.get_render_option()
    opt.point_size = point_size

    while True:          
        # Capture frame-by-frame
        status = camera1.requestFrame(frame)
        if not status:
            print("camera1.requestFrame() failed with status: ", status)
       
        frame_count = frame_count + 1
        elapsed_time = time.time() - start_time

        if elapsed_time >= 5.0:
            fps = frame_count / elapsed_time
            print("FPS: ", fps)
            frame_count = 0
            start_time = time.time()

        # Process the frame
        ab_frame = np.array(frame.getData("ab"), dtype="uint16", copy=False)
        xyz_map = np.array(frame.getData("xyz"), dtype="int16", copy=False)
        
        # Downsample the images
        ab_frame_downsampled = ab_frame[::reduction_factor, ::reduction_factor]
        xyz_downsampled = xyz_map[::reduction_factor, ::reduction_factor, :]

        # Apply scaling to the IR image
        if apply_scaling == 1:
            ab_image = ApplyLogScaling(ab_frame_downsampled)
            ab_image = np.uint8(distance_scale_ir * ab_image)        
        else:
            ab_image = np.uint8(distance_scale_ir * ab_frame_downsampled)

        # Create an RGB image from the AB frame.
        ab_image_rgb = cv.cvtColor(ab_image, cv.COLOR_GRAY2RGB)

        # Perform Pose Tracking
        if track_pose:
            ab_image_rgb = TrackSkeletonPose(ab_image_rgb)

        # Perform Hand Tracking
        if track_hand:
            ab_image_rgb = TrackHandMovement(ab_image_rgb)

        # Create Point cloud
        total_xyz_points = np.resize(xyz_downsampled, (int(xyz_downsampled.shape[0])*xyz_downsampled.shape[1], 3))
        colors = np.resize(ab_image_rgb, (total_xyz_points.shape[0], 3)) 

        # Remove points that are more than 1 meter from the camera
        z_values = total_xyz_points[:, 2]
        points_to_remove = np.where(z_values > 1000)

        filtered_points = np.delete(total_xyz_points, points_to_remove, axis=0)
        filtered_colors = np.delete(colors, points_to_remove, axis=0)

        point_cloud.points = o3d.utility.Vector3dVector(filtered_points)              
        point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors/255)

        # Apply Transform
        point_cloud.transform([[1,0,0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
       
        # Update the visualizer
        if first_time_renderer == 1:
            visualizer.add_geometry(point_cloud)
            first_time_renderer = 0
        visualizer.update_geometry(point_cloud)
        visualizer.poll_events()
        visualizer.update_renderer()

        if cv.waitKey(1) != -1:  # Check if a key is pressed
            print("Exiting the loop.: ")
            break
           
    camera1.stop()
    cv.destroyAllWindows()
    visualizer.destroy_window()
