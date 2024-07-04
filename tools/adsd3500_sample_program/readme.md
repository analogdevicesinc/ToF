# NXP Eval Kit Example program to Capture Frames from the Imager.

This is an Example program that shows the usage of ADSD3500, device driver and Depth-Compute Library.

The example program performs the following functionalities.

1. Resets the ADSD3500 device.
2. Configures the ADSD3500 and depth compute library with the ini file.
3. Configures depth compute library with CCB parameters from the ADSD3500.
4. Sets up the interrupt support.
5. Sets the imaging mode.
6. Starts streaming.
7. Receives frames.
8. Passes the received frames to the depth compute library.
9. Saves the AB, Depth, confidence, XYZ and metadata to the file system.
10. Stops streaming.
11. Closes the camera
12. Exits

# Build Instructions

The Example program can be built with two options, one is by using the Open-source Depth-Compute files and the other is
by using Closed-Source Depth Compute shared object library files. 

## Build using Open-Source Depth Compute Files

To build the program with Open-source Depth Compute Files, run the following command. 

    $ make OPEN_SOURCE_MODE=1  

## Build using Closed-Source Depth Compute Library

To build the program with Close-source Depth Compute Libraries, follow the instructions below. 

1. Copy the .so files to the /usr/lib/ location.

    To use the Fixed Point Depth Compute Library:  
    $ sudo cp depthComputeLibrary/closedSourceLibs/fixedPoint/* /usr/lib/ 

    To use the Fixed Point Depth Compute Library:  
    $ sudo cp depthComputeLibrary/closedSourceLibs/fixedPoint/* /usr/lib/ If Floating Point Depth Compute Library to be used. 

2. Build the program.  

    $ make FLOAT_LIBS=0 (for fixed point library case)

    $ make FLOAT_LIBS=1 (for floating point library case)

## Usage

1. Once the executable is generated, run the application by the following command.

    $ sudo ./run_adsd3500 -m "mode number" -n "number of frames" 

2. The program stores the AB, Depth, Confidence, XYZ frames and metadata in the same location.

## Visualization

1. To visualize the frames generated, transfer the .bin files to a Host PC via SCP connection.

2. Use the python scripts in the visualize folder to visualize the AB, Depth, Confidence and the Point cloud from the XYZ frames as shown below.

    $ python visualize "width" "height" "num_frames" 

    $ python visualize_pointcloud "width" "height" "num_frames" "xyz_frame_path" 



