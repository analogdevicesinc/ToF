# Time of Flight Viewer by Analog Devices, Inc

## RAW Format for Recording and Playback

The Analog Devices, Inc RAW format records and playsback processed data streams such as Active Brightness and Depth view. The recorded file has a very small global header that contains the following elements in this order:

* Frame Height (in pixels) size of 4 bytes
* Frame Width (in pixels) size of 4 bytes
* Number of total frames size of 4 bytes

After this header, the file contains an interleaved data between Active Brightness and Depth view for each frame. A frame is a collection of data of size of Width (in pixels) x Height (in pixels). For example, if an image is size of 1024 pixels x 1024 pixels, then a frame is the entire image of size 1024 pixels x 1024 pixels. Each pixel is of size 2 bytes.

The interleaved data can be better explained by the following example:
* Frame 1 = [Active Brightness1 (1024x1024), Depth View1 (1024x1024)]
* Frame 2 = [Active Brightness2 (1024x1024), Depth View2 (1024x1024)]

...

* Frame n = [Active Brightnessn (1024x1024), Depth Viewn (1024x1024)]

In summary, the whole document can be read as follows:

Raw File =  
[ (Frame Height) (Frame Width) (Number of total frames)  
(Active Brightness 1) (Depth View 1)  
(Active Brightness 2) (Depth View 2)  
(Active Brightness 3) (Depth View 3)  
(Active Brightness n) (Depth View n)] 
