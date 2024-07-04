import numpy as np
import open3d as o3d
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('width', type=int, help='width of the image')
parser.add_argument('height', type=int, help='height of the image')
parser.add_argument('num_frames', type=int, help='number of frames')
parser.add_argument('xyz_file', type=str, help='path to xyz file')

def visualize_pointcloud(args):
    height = args.height
    width = args.width
    pcd = o3d.geometry.PointCloud()
    with open(args.xyz_file) as file:
        byte_array = np.fromfile(file, dtype=np.int16, count=height*width*3)
        cloud = np.reshape(byte_array, [width*height,3])
        # From numpy to Open3D
        pcd.points = o3d.utility.Vector3dVector(cloud)

    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    args = parser.parse_args()
    visualize_pointcloud(args)