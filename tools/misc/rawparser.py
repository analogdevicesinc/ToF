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
#import aditofpython as tof
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import sys
import os
import time
import argparse
import cv2 as cv
import open3d as o3d

width = 0
height = 0
frameNumber = 0
bytePerPx = 10
startOfFrame = 0
raw_img_dir = '\\sample_raw\\'
fileName = 'frames202309111819'
rawFileType = '.RAW'
binFileType = '.bin'
pngFileType = '.png'
plyFileType = '.ply'

def visualize_ab(filename,index):
    ab_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the AB data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16)
        ab_frame = np.reshape(byte_array[height*width*0:height*width*1], [height,width])

        #normalize ab data to 8bit image
        norm_ab_frame = cv.normalize(ab_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        norm_ab_frame = np.uint8(norm_ab_frame)
        norm_ab_frame = cv.cvtColor(norm_ab_frame, cv.COLOR_GRAY2RGB)
        
        #save depth frame as
        img = o3d.geometry.Image(norm_ab_frame)
        #Save the image to a file
        o3d.io.write_image(index+ 'ab_' + args.filename + pngFileType, img)

def visualize_depth(filename,index):
    ab_frame = np.zeros([height,width])
    with open ('%s' % filename) as file:
        #parse the depth data from binary file 
        byte_array = np.fromfile(file, dtype=np.uint16)
        depth_frame = np.reshape(byte_array[height*width*1:height*width*2], [height,width])

        #normalize depth data to 8bit image
        norm_depth_frame = cv.normalize(depth_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
        norm_depth_frame = np.uint8(norm_depth_frame)
        norm_depth_frame = cv.applyColorMap(norm_depth_frame,cv.COLORMAP_TURBO)
        
        #save depth frame as
        img = o3d.geometry.Image(norm_depth_frame)
        #Save the image to a file
        o3d.io.write_image(index+ 'depth_' + args.filename + pngFileType, img)

def visualize_pcloud(filename,index):
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud", 1200, 1200)

    point_cloud = o3d.geometry.PointCloud()
    xyz_frame = np.zeros([height*width,3])
    with open ('%s' % filename) as file:
        byte_array = np.fromfile(file, dtype=np.int16)
        xyz_frame = np.resize(byte_array[height*width*2:height*width*5], (int(len(byte_array[height*width*2:height*width*3])),3))

        point_cloud.points = o3d.utility.Vector3dVector(xyz_frame)
        point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.io.write_point_cloud(index+ 'pointcloud_' + args.filename + plyFileType,point_cloud)
        vis.add_geometry(point_cloud)
        vis.update_geometry(point_cloud)
        vis.run()
        vis.poll_events()
        vis.update_renderer()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Scrip to parse a raw file and extract different frame data ')
    parser.add_argument("--filename", default = 'frames202309111819', help="filename to parse")
    args = parser.parse_args()
    print(f"filename: {args.filename}")
    dir_path = os.path.dirname( os.path.abspath(__file__)) + raw_img_dir
    if os.path.exists(dir_path):
        print(f"The directory {dir_path} already exists.")
    else:
        # Create the directory
        os.makedirs(dir_path)
        print(f"The directory {dir_path} was created.")
    
    #identify width, height and number of frames from raw file
    with open(os.path.dirname( os.path.abspath(__file__))+ '\\' + args.filename + rawFileType, 'rb') as f:
        data = f.read(12)
        width = int.from_bytes(data[:4], 'little')
        height = int.from_bytes(data[4:8], 'little')
        frameNumber = int.from_bytes(data[8:12], 'little')
        print(f"width: {width} height: {height} # of frames:  {frameNumber}")
        #show frame details
        file_size = os.path.getsize(os.path.dirname( os.path.abspath(__file__)) + '\\' + args.filename + rawFileType)
        print("file size: " + str(file_size))
        sizeOfHeader = 12;
        sizeOfFrame = bytePerPx * height * width;
        print("frame size: " + str(sizeOfFrame))
        m_numberOfFrames = (file_size - sizeOfHeader) / sizeOfFrame;
        print("number of frames: " + str(m_numberOfFrames))
        f.seek(12)
        data = f.read(file_size - sizeOfHeader)
        m_frameData = np.frombuffer(data, dtype=np.uint8)

    #create directory for output frames
    new_dir = os.path.dirname( os.path.abspath(__file__)) + raw_img_dir + args.filename
    print(new_dir)
    os.mkdir(new_dir)
    first_time_render_pc = 1
    for i in range(0, frameNumber):
        # Create frame folders
        frameDir = new_dir + '\\' + args.filename + '_' + str(i) +'\\'
        os.mkdir(frameDir)
        binFileName = frameDir + args.filename + '_' + str(i) + binFileType
        print(binFileName)
        # Open the file in binary mode
        with open(binFileName , "wb") as f:
            endOfFrame = startOfFrame +sizeOfFrame
            f.write(m_frameData[startOfFrame : endOfFrame])
            startOfFrame = endOfFrame
        visualize_ab(binFileName,frameDir)
        visualize_depth(binFileName,frameDir)
        if first_time_render_pc:
            visualize_pcloud(binFileName,frameDir)
            first_time_render_pc = 0
   