#!/usr/bin/env python3
# BSD 3-Clause License (see original notice above)

import numpy as np
import sys
import os
import argparse
import cv2 as cv
import open3d as o3d
import struct
import re

# ---- CONSTANTS ----
MEGA_PIXEL = 1024
QMEGA_PIXEL = [512, 640, 256, 320]
PNG_EXT = '.png'
PLY_EXT = '.ply'
METADATA_LENGTH = 128
RAW_PARSER_VERSION = "1.1.0"
DEFAULT_CONF_BYTES = 4
DEFAULT_AB_BYTES = 2
DEFAULT_XYZ_BYTES = 6
DEFAULT_DEPTH_BYTES = 2

def make_dir(path):
    os.makedirs(path, exist_ok=True)

def safe_join(*args):
    return os.path.join(*args)

def parse_frame_range(range_str, total_frames):
    """
    Accepts range_str (e.g. '30', '30-', '30-40') and total_frames.
    Returns (start, end) inclusive.
    """
    if not range_str:
        return 0, total_frames - 1
    match = re.fullmatch(r'(\d+)(?:-(\d*)?)?', range_str)
    if not match:
        raise ValueError(f"Invalid frame range: {range_str}")
    start = int(match.group(1))
    end = match.group(2)
    if match.group(2) is not None:
        # It's a range: '10-', '10-15'
        if end == '' or end is None:
            end_idx = total_frames - 1
        else:
            end_idx = int(end)
    else:
        # Single frame: '10'
        end_idx = start
    # Clamp to valid frame indices
    start = max(start, 0)
    end_idx = min(end_idx, total_frames - 1)
    if start > end_idx:
        raise ValueError(f"Start frame {start} greater than end frame {end_idx}")
    return start, end_idx

def generate_metadata(filename, directory, base_filename, index):
    elemsList = [
        ('frameWidth',             'H'),
        ('frameHeight',            'H'),
        ('outputconfig',           'B'),
        ('depthPhaseBits',         'B'),
        ('ABBits',                 'B'),
        ('confidenceBits',         'B'),
        ('invalidPhaseValue',      'H'),
        ('frequencyIndex',         'B'),
        ('ABFrequencyIndex',       'B'),
        ('frameNumber',            'L'),
        ('imagerMode',             'B'),
        ('noOfPhases',             'B'),
        ('noOfFrequencies',        'H'),
        ('ElapsedTimeinFrac',      'L'),
        ('ElapsedTimeinSec',       'L'),
        ('sensorTemp',             'L'),
        ('laserTemp',              'L'),
        ('paddingBytes',         '92x'),
    ]
    format_string = '=' + ''.join(fmt for _, fmt in elemsList)
    with open(filename, "rb") as file:
        data = file.read(METADATA_LENGTH)
        values = struct.unpack(format_string, data)
        elements = [(name, value) for (name, _), value in zip(elemsList, values)]
        meta_path = safe_join(directory, f'metadata_{base_filename}_{index}.txt')
        with open(meta_path, 'w') as outfile:
            outfile.writelines([f"{i}\n" for i in elements])

def generate_depth(filename, directory, base_filename, index, width, height):
    with open(filename, "rb") as file:
        file.seek(METADATA_LENGTH)
        byte_array = np.fromfile(file, dtype=np.uint16, count=height * width)
    depth_frame = np.reshape(byte_array, (height, width))
    norm_depth = cv.normalize(depth_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
    color_depth = cv.applyColorMap(norm_depth, cv.COLORMAP_TURBO)
    img = o3d.geometry.Image(color_depth)
    o3d.io.write_image(safe_join(directory, f'depth_{base_filename}_{index}{PNG_EXT}'), img)

def visualize_ab(filename, directory, base_filename, index, width, height, depth_bytes, log_image = False):
    offset = depth_bytes * height * width + METADATA_LENGTH
    with open(filename, "rb") as file:
        file.seek(offset)
        byte_array = np.fromfile(file, dtype=np.uint16, count=height * width)
    ab_frame = np.reshape(byte_array, (height, width))
    norm_ab = cv.normalize(ab_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
    if log_image:
        c = 255 / np.log(1 + np.max(norm_ab))
        norm_ab = c * (np.log(norm_ab + 1))
        norm_ab = np.array(norm_ab, dtype=np.uint8)
    else:
        norm_ab = np.uint8(norm_ab)
    norm_ab = cv.cvtColor(norm_ab, cv.COLOR_GRAY2RGB)
    img = o3d.geometry.Image(norm_ab)
    o3d.io.write_image(safe_join(directory, f'ab_{base_filename}_{index}{PNG_EXT}'), img)

def generate_confidence(filename, directory, base_filename, index, width, height, depth_bytes, ab_bytes):
    offset = (depth_bytes + ab_bytes) * height * width + METADATA_LENGTH
    with open(filename, "rb") as file:
        file.seek(offset)
        byte_array = np.fromfile(file, dtype=np.int16, count=height * width)
    conf_frame = np.reshape(byte_array, (height, width))
    norm_conf = cv.normalize(conf_frame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
    norm_conf = 255 - np.uint8(norm_conf)
    img = o3d.geometry.Image(norm_conf)
    o3d.io.write_image(safe_join(directory, f'conf_{base_filename}_{index}{PNG_EXT}'), img)

def visualize_pcloud(filename, directory, base_filename, index, width, height, qmegapixel, mega_pixel, conf_bytes, depth_bytes, ab_bytes, xyz_bytes):
    if width in qmegapixel and height in qmegapixel:
        offset = (conf_bytes + depth_bytes + ab_bytes) * height * width + METADATA_LENGTH
    elif width == mega_pixel and height == mega_pixel:
        offset = (depth_bytes + ab_bytes) * height * width + METADATA_LENGTH
    else:
        return
    with open(filename, "rb") as file:
        file.seek(offset)
        byte_array = np.fromfile(file, dtype=np.int16, count=height * width * 3)
    if byte_array.size != height * width * 3:
        return
    xyz_frame = np.resize(byte_array, (height * width, 3))
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(xyz_frame)
    point_cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.io.write_point_cloud(safe_join(directory, f'pointcloud_{base_filename}_{index}{PLY_EXT}'), point_cloud)

def generate_vid(main_dir, base_filename, processed_frames, width, height):
    vid_dir = safe_join(main_dir, f'vid_{base_filename}')
    make_dir(vid_dir)
    video_path = safe_join(vid_dir, f'vid_{base_filename}.mp4')
    video = cv.VideoWriter(video_path, cv.VideoWriter_fourcc(*"mp4v"), 10, (width * 2, height))
    for i in processed_frames:
        frame_dir = safe_join(main_dir, f"{base_filename}_{i}")
        depth_img = cv.imread(safe_join(frame_dir, f'depth_{base_filename}_{i}{PNG_EXT}'))
        ab_img = cv.imread(safe_join(frame_dir, f'ab_{base_filename}_{i}{PNG_EXT}'))
        if depth_img is None or ab_img is None:
            continue
        new_img = cv.hconcat([depth_img, ab_img])
        new_img = cv.resize(new_img, (width * 2, height))
        video.write(new_img)
    video.release()

def main():
    parser = argparse.ArgumentParser(
        description='Script to parse a raw file and extract different frame data')
    parser.add_argument("filename", type=str, help="bin filename to parse")
    parser.add_argument("-o", "--outdir", type=str, default=None, help="Output directory (optional)")
    parser.add_argument("-n", "--no_xyz", dest='no_xyz', action='store_true', help="Input file doesn't have XYZ data")
    parser.add_argument("-f", "--frames", type=str, default=None,
                        help="Frame range: N (just N), N- (from N to end), N-M (N to M inclusive)")
    args = parser.parse_args()

    conf_bytes = DEFAULT_CONF_BYTES
    ab_bytes = DEFAULT_AB_BYTES
    xyz_bytes = 0 if args.no_xyz else DEFAULT_XYZ_BYTES
    depth_bytes = DEFAULT_DEPTH_BYTES

    if not os.path.exists(args.filename):
        sys.exit(f"Error: {args.filename} does not exist")

    print(f"rawparser {RAW_PARSER_VERSION}\nfilename: {args.filename}")

    base_filename, _ = os.path.splitext(os.path.basename(args.filename))
    if args.outdir:
        dir_path = os.path.abspath(args.outdir)
    else:
        base_dir, _ = os.path.splitext(args.filename)
        dir_path = base_dir

    if os.path.exists(dir_path):
        print(f"The directory {dir_path} already exists.")
        sys.exit(1)
    else:
        make_dir(dir_path)
    print(f"The directory {dir_path} was created.")

    # --- Read header and info ---
    with open(args.filename, 'rb') as f:
        data = f.read(8)
        width = int.from_bytes(data[:2], 'little')
        height = int.from_bytes(data[2:4], 'little')
        bitsperpixcel = data
        print(f"Width x Height: {width}px x {height}px")
        if not (bitsperpixcel[5]):
            depth_bytes = 0
        if not (bitsperpixcel[6]):
            ab_bytes = 0
        if not (bitsperpixcel[7]):
            conf_bytes = 0
        print(f"Bits in depth: {depth_bytes}")
        print(f"Bits in AB: {ab_bytes}")
        print(f"Bits in conf: {conf_bytes}")

        file_size = os.path.getsize(args.filename)
        print(f"File size: {file_size}")

        if width in QMEGA_PIXEL and height in QMEGA_PIXEL:
            byte_per_px = ab_bytes + depth_bytes + conf_bytes + xyz_bytes
        elif width == MEGA_PIXEL and height == MEGA_PIXEL:
            byte_per_px = ab_bytes + depth_bytes + xyz_bytes
        else:
            sys.exit("Error: no byte per pixel data on this file, cannot parse.")
        size_of_frame = (byte_per_px * height * width) + METADATA_LENGTH
        print(f"Frame size: {size_of_frame}")
        num_frames = file_size // size_of_frame
        print(f"Relative Frame Range: 0 to {num_frames-1}")
        f.seek(0)
        m_frame_data = np.frombuffer(f.read(file_size), dtype=np.uint8)

    # --- Parse frame range argument ---
    try:
        start_frame, end_frame = parse_frame_range(args.frames, num_frames)
    except Exception as e:
        sys.exit(f"Invalid --frames argument: {e}")

    # --- Frame Processing Loop ---
    processed_frames = []
    for i in range(num_frames):
        if i < start_frame or i > end_frame:
            continue
        print(f"Processing frame #: {i}", end='\r', flush=True)
        frame_dir = safe_join(dir_path, f"{base_filename}_{i}")
        make_dir(frame_dir)
        bin_filename = safe_join(frame_dir, f"{base_filename}_{i}.bin")
        start_of_frame = i * size_of_frame
        end_of_frame = start_of_frame + size_of_frame
        with open(bin_filename, "wb") as f:
            f.write(m_frame_data[start_of_frame:end_of_frame])
        generate_metadata(bin_filename, frame_dir, base_filename, str(i))
        if depth_bytes:
            generate_depth(bin_filename, frame_dir, base_filename, str(i), width, height)
        if ab_bytes:
            visualize_ab(bin_filename, frame_dir, base_filename, str(i), width, height, depth_bytes, False)
        if (width in QMEGA_PIXEL and height in QMEGA_PIXEL) and conf_bytes:
            generate_confidence(bin_filename, frame_dir, base_filename, str(i), width, height, depth_bytes, ab_bytes)
        if xyz_bytes:
            visualize_pcloud(bin_filename, frame_dir, base_filename, str(i), width, height, QMEGA_PIXEL, MEGA_PIXEL, conf_bytes, depth_bytes, ab_bytes, xyz_bytes)
        processed_frames.append(i)

    if processed_frames:
        generate_vid(dir_path, base_filename, processed_frames, width, height)

if __name__ == "__main__":
    main()
