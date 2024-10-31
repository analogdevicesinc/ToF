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
import argparse
import numpy as np
import cv2 as cv
import open3d as o3d
from enum import Enum
import sys

WINDOW_NAME_DEPTH = "Display Depth"
WINDOW_NAME_COLOR = "Display Color"

def transform_image(np_image):
    return o3d.geometry.Image(np_image)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to run PointCloud')
    parser.add_argument('-ip', '--ip', help='Ip address of the ToF device')
    parser.add_argument('-f', '--frame', help='Name of an acquired frame to be used')
    parser.add_argument('-m', '--mode', help='Camera mode')

    args = parser.parse_args()
    system = tof.System()

    print("SDK version: ", tof.getApiVersion(), " | branch: ", tof.getBranchVersion(), " | commit: ", tof.getCommitVersion())
    
    cameras = []
    ip = ''
    fileName = args.frame
    mode = 0

    if args.mode is not None:
        mode = int(args.mode)

    if args.ip is not None:
        ip = 'ip:' + args.ip

    if fileName is None:
        status = system.getCameraList(cameras, ip)
        print("system.getCameraList()", status)

        camera1 = cameras[0]

        status = cameras[0].initialize()
        if not status:
            print("cameras[0].initialize() failed with status: ", status)

        modes = []
        status = cameras[0].getAvailableModes(modes)
        if not status:
            print("system.getAvailableModes() failed with status: ", status)

        if int(mode) not in modes:
            print(f"Error: Unknown mode - {mode}")
            print("Available modes: ", modes)
            exit(-3)

        status = cameras[0].setMode(mode)
        if not status:
            print("cameras[0].setMode() failed with status:", status)

        camDetails = tof.CameraDetails()
        status = cameras[0].getDetails(camDetails)
        if not status:
            print("system.getDetails() failed with status: ", status)

        status = camera1.start()
        print("camera1.start()", status)

        # Get the first frame for details
        frame = tof.Frame()
        status = cameras[0].requestFrame(frame)

        frameDataDetails = tof.FrameDataDetails()
        status = frame.getDataDetails("depth", frameDataDetails)
        width = frameDataDetails.width
        height = frameDataDetails.height

        # Get intrinsic parameters from camera
        intrinsicParameters = camDetails.intrinsics
        fx = intrinsicParameters.fx
        fy = intrinsicParameters.fy
        cx = intrinsicParameters.cx
        cy = intrinsicParameters.cy
        cameraIntrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

        # Get camera details for frame correction
        # TO DO: Get the range from camera details when it will be defined
        camera_range = 4096
        bitCount = 12
        max_value_of_AB_pixel = 2 ** bitCount - 1
        distance_scale_ab = 255.0 / max_value_of_AB_pixel
        distance_scale = 255.0 / camera_range

        # Create visualizer for depth and ab
        vis_depth = o3d.visualization.Visualizer()
        vis_depth.create_window("Depth", 2 * width, 2 * height)

        vis_ab = o3d.visualization.Visualizer()
        vis_ab.create_window("AB", 2 * width, 2 * height)

        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window("PointCloud", 1200, 1200)
        first_time_render_pc = 1
        point_cloud = o3d.geometry.PointCloud()

        while True:
            # Capture frame-by-frame
            status = cameras[0].requestFrame(frame)
            if not status:
                print("cameras[0].requestFrame() failed with status: ", status)
                
            depth_map = np.array(frame.getData("depth"), dtype="uint16", copy=False)
            ab_map = np.array(frame.getData("ab"), dtype="uint16", copy=False)
            xyz_map = np.array(frame.getData("xyz"), dtype="int16", copy=False)

            # Create the AB image
            ab_map = ab_map[0: int(ab_map.shape[0]), :]
            ab_map = distance_scale_ab * ab_map
            ab_map = np.uint8(ab_map)
            ab_map = cv.cvtColor(ab_map, cv.COLOR_GRAY2RGB)

            # Show AB image
            vis_ab.add_geometry(transform_image(ab_map))
            vis_ab.poll_events()

            # Create the Depth image
            xyz_points = np.resize(xyz_map, (int(depth_map.shape[0]) * depth_map.shape[1], 3))
            depth_map = depth_map[0: int(depth_map.shape[0]), :]
            depth_map = distance_scale * depth_map
            depth_map = np.uint8(depth_map)
            depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_WINTER)

            # Show depth image
            vis_depth.add_geometry(transform_image(depth_map))
            vis_depth.poll_events()

            # Show the point cloud
            point_cloud.points = o3d.utility.Vector3dVector(xyz_points)
            point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            if first_time_render_pc:
                vis.add_geometry(point_cloud)
                first_time_render_pc = 0
            vis.update_geometry(point_cloud)
            vis.poll_events()
            vis.update_renderer()

            if cv.waitKey(1) >= 0:
                break
    else:
        # Reading binary file 
        frame = tof.Frame()
        frameHandler = tof.FrameHandler()
        status = frameHandler.readNextFrame(frame, fileName)
        if not status:
            print('Failed to read frame with status: ', status)

        frameDataDetails = tof.FrameDataDetails()
        status = frame.getDataDetails("depth", frameDataDetails)
        width = frameDataDetails.width
        height = frameDataDetails.height

        # Get camera details for frame correction
        # TO DO: Get the range from camera details when it will be defined
        camera_range = 4096
        bitCount = 12
        max_value_of_AB_pixel = 2 ** bitCount - 1
        distance_scale_ab = 255.0 / max_value_of_AB_pixel
        distance_scale = 255.0 / camera_range

        # Create visualizer for depth and ab
        vis_depth = o3d.visualization.Visualizer()
        vis_depth.create_window("Depth", 2 * width, 2 * height)

        vis_ab = o3d.visualization.Visualizer()
        vis_ab.create_window("AB", 2 * width, 2 * height)

        # Create visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window("PointCloud", 1200, 1200)
        first_time_render_pc = 1
        point_cloud = o3d.geometry.PointCloud()

        while True:

            depth_map = np.array(frame.getData("depth"), dtype="uint16", copy=False)
            ab_map = np.array(frame.getData("ab"), dtype="uint16", copy=False)
            xyz_map = np.array(frame.getData("xyz"), dtype="int16", copy=False)

            # Create the AB image
            ab_map = ab_map[0: int(ab_map.shape[0]), :]
            ab_map = distance_scale_ab * ab_map
            ab_map = np.uint8(ab_map)
            ab_map = cv.cvtColor(ab_map, cv.COLOR_GRAY2RGB)

            # Show AB image
            vis_ab.add_geometry(transform_image(ab_map))
            vis_ab.poll_events()

            # Create the Depth image
            xyz_points = np.resize(xyz_map, (int(depth_map.shape[0]) * depth_map.shape[1], 3))
            depth_map = depth_map[0: int(depth_map.shape[0]), :]
            depth_map = distance_scale * depth_map
            depth_map = np.uint8(depth_map)
            depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_WINTER)

            # Show depth image
            vis_depth.add_geometry(transform_image(depth_map))
            vis_depth.poll_events()

            # Show the point cloud
            point_cloud.points = o3d.utility.Vector3dVector(xyz_points)
            point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            if first_time_render_pc:
                vis.add_geometry(point_cloud)
                first_time_render_pc = 0
            vis.update_geometry(point_cloud)
            vis.poll_events()
            vis.update_renderer()

            if cv.waitKey(1) >= 0:
                break