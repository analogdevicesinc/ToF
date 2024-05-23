# Linux Example program

This is a Linux example program that shows the usage of ADSD3500, device driver and Depth-Compute Library.

The example program performs the following functionalities.

1. Resets the ADSD3500 device.
2. Configures the ADSD3500 and depth compute library with the ini file.
3. Configures depth compute library with CCB parameters from the ADSD3500.
4. Sets up the interrupt support.
5. Sets the imaging mode.
6. Starts streaming.
7. Receives frames.
8. Passes the received frames to the depth compute library.
9. Saves the AB, Depth, confidence and metadata to the file system.
10. Stops streaming.
11. Closes the camera
12. Exits

## Install Depth Compute Library files.

1. Navigate to the current project directory in the command line terminal.

2. Copy the Depth Compute Library files to the path /usr/lib/

    $ cd depthComputeLibrary/libs/
    $ sudo cp libtofi_compute.so libtofi_config.so /usr/lib/

## Usage

1. Navigate to the current project directory in the command line terminal.

2. Run the following command to generate executable for the example program.

    $ make 

3. Once the executable is generated, run the application by the following command.

    $ sudo ./run_adsd3500 -m <mode number> -n <number of frames>



