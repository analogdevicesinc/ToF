# Raw File Parser Example

### Overview
This example takes raw file as an input and extracts the depth absolute brightness and point cloud data.
depth frame and AB frame are  saved as `.png` while point cloud data is saved as `.ply` 
An `.mp4` file is also generated showing  both AB and depth frames.
metadata is saved as a text file.

Usage:
- put raw recording files in the same directory of the `rawparser.py`
- run `python rawparser.py --filename [RAW_RECORDING_FILENAME]` (file extension `.raw` excluded)

`sample_raw` folder will be created
first point cloud frame 3d image will be visualized. Close the visualize window to proceed saving all data
`.png` files for each frame of AB and Depth will be found in `sample_raw/[RAW_RECORDING_FILENAME]`

### Dependencies
```
pip install opencv-python
pip install numpy
pip install open3d
pip install argparse
```
