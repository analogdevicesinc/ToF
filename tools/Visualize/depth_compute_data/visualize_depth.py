import numpy as np
import matplotlib.pyplot as plt
import sys

width = 1024
height = 1024
try:
	inputfile = sys.argv[1]
	if 3 <= len(sys.argv) - 1:
		width = int(sys.argv[2])
		height = int(sys.argv[3])
except:
	print ('USAGE:')
	print ('visualize_depth.py <input-file-name>')
	print ('visualize_depth.py <input-file-name> <height> <width>')
	sys.exit(2)

print ('Input file is: ', inputfile)

depth_frame = np.zeros([height,width])

with open ('%s' % inputfile) as file:
    byte_array = np.fromfile(file, dtype=np.uint16)
    depth_frame = np.reshape(byte_array[height*width*0:height*width*1], [height,width])

plt.figure()
plt.imshow(depth_frame, cmap = 'jet')
plt.colorbar()
plt.show()

input()
