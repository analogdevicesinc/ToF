# Visualize all
import numpy as np
import struct
import sys
import argparse
import matplotlib.pyplot as plt

# Converts list of bytes one int
def combine_bytes(bytes_list):
    num = 0
    for byte in bytes_list:
        if byte > 255:
            print('Byte value is not less than 255')
            return
        num = (num << 8) + byte
        
    return num 

def get_deinterleaved_depthabconf(height, width, bin_file_path, frames = 1, depth_bits = 16, ab_bits = 16, conf_bits = 8):
    frame_size = height*width

    total_num_bits = depth_bits+ab_bits+conf_bits
    if (total_num_bits)%8 > 0:
        print('Total number of bits are not compatible with interleaved format please check parameters')
        quit()

    totalbytes = int(total_num_bits/8)
    bytenum = totalbytes
    
    depth_buf = []
    ab_buf = []
    conf_buf = []

    ab_frame = np.zeros([height,width, frames])
    depth_frame = np.zeros([height,width,frames])
    conf_frame = np.zeros([height,width,frames])
    
    fcount = 0
    pix_count = 0

    with open ('%s' % bin_file_path, mode='rb') as file:
        fileContent = file.read()
        filesize = len(fileContent)
        print(filesize)
        
        while bytenum <= filesize:
            byte_int_array = struct.unpack(str(totalbytes)+'B', fileContent[bytenum-5:bytenum])
            interleaved_pixel_data = combine_bytes(byte_int_array)
            
            ab_mask = 0
            for i in range(ab_bits):
                ab_mask = (ab_mask << 1) + 1
            ab_v = interleaved_pixel_data & ab_mask
            
            # If > 1 byte convert to little endian
            if ab_bits > 8:
                byte1 = ab_v & 0xff
                byte0 = ab_v >> 8
                ab_v = (byte1 << 8) + byte0
            
            interleaved_pixel_data = interleaved_pixel_data >> ab_bits
            
            conf_mask = 0
            for i in range(conf_bits):
                conf_mask = (conf_mask << 1) + 1
            conf_v = interleaved_pixel_data & conf_mask

            interleaved_pixel_data = interleaved_pixel_data >> conf_bits
            
            depth_mask = 0
            for i in range(depth_bits):
                depth_mask = (depth_mask << 1) + 1
            depth_v = interleaved_pixel_data & depth_mask
            
            # If > 1 byte convert to little endian
            if ab_bits > 8:
                byte1 = depth_v & 0xff
                byte0 = depth_v >> 8
                depth_v = (byte1 << 8) + byte0
            
            bytenum = bytenum + 5
            pix_count = pix_count + 1

            depth_buf.append(depth_v)
            ab_buf.append(ab_v)
            conf_buf.append(conf_v)

            if pix_count == frame_size:
                pix_count = 0
                depth_frame[:,:,fcount] = np.reshape(depth_buf, [height,width])
                ab_frame[:,:,fcount] = np.reshape(ab_buf, [height,width])
                conf_frame[:,:,fcount] = np.reshape(conf_buf, [height,width])
                fcount = fcount + 1
                depth_buf=[]
                ab_buf=[]
                conf_buf_temp = conf_buf
                conf_buf=[]
    
    return depth_frame, ab_frame, conf_frame

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
                    prog = 'visualize_interleaved',
                    description = 'Deinterleaved adsd3500 output and generates pngs')
    
    parser.add_argument('file_path', action='store', default='temp', help='location of source frame')
    parser.add_argument('--height', action='store', default=512, help='frame height')
    parser.add_argument('--width', action='store', default=512, help='frame width')
    parser.add_argument('--frames', action='store', default=1, help='number of frames in .bin file')
    parser.add_argument('--depth_bits', action='store', default=16, help='number of depth bits')
    parser.add_argument('--ab_bits', action='store', default=16, help='number of ab bits')
    parser.add_argument('--conf_bits', action='store', default=8, help='number of conf bits')

    args = parser.parse_args()
    print(type(args.height), args.depth_bits, args.ab_bits)

    depth_frame, ab_frame, conf_frame = get_deinterleaved_depthabconf(args.height, args.width, args.file_path, frames = args.frames, depth_bits = args.depth_bits, ab_bits = args.ab_bits, conf_bits = args.conf_bits)

    plt.imshow(depth_frame[:,:,0], cmap='rainbow')
    plt.savefig('depth.png')
    plt.imshow(ab_frame[:,:,0], cmap='gray')
    plt.savefig('ab.png')
    plt.imshow(conf_frame[:,:,0], cmap='hot')
    plt.savefig('conf.png')

