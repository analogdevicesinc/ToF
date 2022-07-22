# Time of Flight Viewer by Analog Devices, Inc

## FSF Format for Recording and Playback

The Analog Devices, Inc FSF format records and playsback processed data streams such as Active Brightness, Depth view, and Point Cloud. The recorded file has a global header that consist on the following elements:

* Header size
* FSF Magic Number
* File format Major Version
* File format Minor Version
* File header size
* Stream information size
* Stream header size
* Optional file header size
* File comment size
* File offset information location
* Number of frames
* Number of streams

Each identified stream type has its own global header with the following elements:

* System ID
* Stream Type (i.e. ACTIVE_BR, DEPTH, X, Y, etc.)
* Channel format
* Bytes per pixel
* Number of rows per stream
* Number of columns per stream
* Optional stream header size
* Stream comment size
* Compression scheme

In every frame the following header is created for each stream type:

* Time Stamp
* Compressed stream size
* Optional stream header
* Stream comment

After this last header, the FSF file assigns the char value for each stream type of size = Height (in pixels) x Width (in pixels) x 2 (bytes).

To playback the FSF file, it should follow the same format. If any of the headers have incomplete information, then it is very likely that the Viewer GUI might not be able to reproduce any valid output.

Please refer to the FSF document [here](https://bitbucket.analog.com/projects/TOFI/repos/utilities/browse/FSF)