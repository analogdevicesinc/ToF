import numpy as np
import matplotlib.pyplot as plt

height = 320
width = 256
num_frames = 10
ab_frame_size = height*width*2 #256*320*2 Bytes or 160KB
depth_frame_size = height*width*2 #256*320*2 Bytes or 160KB
conf_frame_size = height*width #256*320 Bytes or 80KB

ab_array = np.empty((height, width, num_frames), dtype=np.uint16)
depth_array = np.empty((height, width, num_frames), dtype=np.uint16)
conf_array = np.empty((height, width, num_frames), dtype=np.uint8)

ab_file_path = 'out_ab.bin'
depth_file_path = 'out_depth.bin'
conf_file_path = 'out_conf.bin'

ab_bin = open(ab_file_path, "rb")
depth_bin = open(depth_file_path, "rb")
conf_bin = open(conf_file_path, "rb")

for i in range(num_frames):
    ab_data = np.fromfile(ab_bin, dtype=np.uint16, count=height*width).reshape((height, width))
    depth_data = np.fromfile(depth_bin, dtype=np.uint16, count=height*width).reshape((height, width))
    conf_data = np.fromfile(conf_bin, dtype=np.uint8, count=height*width).reshape((height, width))
    
    ab_array[:,:, i] = ab_data
    depth_array[:,:, i] = depth_data
    conf_array[:,:, i] = conf_data


# Visualize the AB, Depth and Confidence images
plt.figure(figsize=(8, 8))
plt.imshow(ab_array[:,:,7], cmap='gray')
plt.title('AB Image')
plt.colorbar(label='Pixel Intensity')
plt.axis('off')

plt.figure(figsize=(8, 8))
plt.imshow(depth_array[:,:,7])
plt.title('Depth Image')
plt.colorbar(label='Pixel Intensity')
plt.axis('off')

plt.figure(figsize=(8, 8))
plt.imshow(conf_array[:,:,7])
plt.title('Confidence Frame')
plt.colorbar(label='Pixel Intensity')
plt.axis('off')
plt.show()