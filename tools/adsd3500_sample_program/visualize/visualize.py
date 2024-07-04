import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('width', type=int, help='width of the image')
parser.add_argument('height', type=int, help='height of the image')
parser.add_argument('num_frames', type=int, help='number of frames')
parser.add_argument('--floatLibs', type=int, default=0, 
    help='The flag is set as 1, if the executable is build with Floating Point Depth Compute Library. \
          If the executable is build with Fixed Point Depth Compute Library, the flag is set as 0')
args = parser.parse_args()

height = args.height
width = args.width
num_frames = args.num_frames
isFloatLibs = args.floatLibs
ab_frame_size = height * width * 2  
depth_frame_size = height * width * 2  
conf_frame_size = height * width * 4  

ab_array = np.empty((height, width, num_frames), dtype=np.uint16)
depth_array = np.empty((height, width, num_frames), dtype=np.uint16)
conf_array = np.empty((height, width, num_frames), dtype=np.float32)

ab_file_path = 'out_ab.bin' # Path to AB frame.
depth_file_path = 'out_depth.bin' # Path to Depth frame.
conf_file_path = 'out_conf.bin' # Path to Confidence frame.

# Open the binary files
with open(ab_file_path, "rb") as ab_bin, open(depth_file_path, "rb") as depth_bin, open(conf_file_path, "rb") as conf_bin:
    for i in range(num_frames):
        # Read AB data
        ab_data = np.fromfile(ab_bin, dtype=np.uint16, count=height * width).reshape((height, width))
        ab_array[:, :, i] = ab_data

        # Read depth data
        depth_data = np.fromfile(depth_bin, dtype=np.uint16, count=height * width).reshape((height, width))
        depth_array[:, :, i] = depth_data

        # Read conf data as uint8
        if isFloatLibs: 
            conf_data = np.fromfile(conf_bin, dtype=np.float32, count=height * width).reshape((height, width))
        else:
            conf_data = np.fromfile(conf_bin, dtype=np.uint16, count=height * width).reshape((height, width))
        conf_array[:, :, i] = conf_data

# Visualize the AB, Depth, and Conf images
for i in range(num_frames):
    plt.figure(figsize=(8, 8))
    plt.imshow(ab_array[:, :, i], cmap='gray')
    plt.title('AB Image')
    plt.colorbar(label='Pixel Intensity')
    plt.axis('off')

for i in range(num_frames):
    plt.figure(figsize=(8, 8))
    plt.imshow(depth_array[:, :, i])
    plt.title('Depth Image')
    plt.colorbar(label='Pixel Intensity')
    plt.axis('off')

plt.show()