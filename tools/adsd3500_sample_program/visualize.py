import numpy as np
import matplotlib.pyplot as plt

height = 320
width = 256

# Read the binary file containing the Depth image data
with open('out_depth.bin', 'rb') as f:
    # Read the data and reshape it into a 2D array
    depth_data = np.fromfile(f, dtype=np.uint16)
    depth_data = depth_data.reshape((height, width))

# Visualize the image
plt.figure(figsize=(8, 8))
plt.imshow(depth_data)
plt.title('Depth Image')
plt.colorbar(label='Pixel Intensity')
plt.axis('off')
plt.show()

# Read the binary file containing the AB Image data
with open('out_ab.bin', 'rb') as f:
    # Read the data and reshape it into a 2D array
    ab_data = np.fromfile(f, dtype=np.uint16)
    ab_data = ab_data.reshape((height, width))

# Visualize the image
plt.figure(figsize=(8, 8))
plt.imshow(ab_data)
plt.title('AB Image')
plt.colorbar(label='Pixel Intensity')
plt.axis('off')
plt.show()