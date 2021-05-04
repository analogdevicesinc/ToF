# Build and Run Application
Note:The location of this README is referred to as ROOTDIR  

## Dependencies
* [CMake](https://cmake.org/)
* Supported C compiler
  * Linux/ OS X: Clang/LLVM 7/GCC
  * Windows: If you have access to a license, install [Visual Studio 2015 with C++ Support](https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2015).  If not, you can install the build tools individually:
    * [Microsoft Build Tools 2015](https://www.microsoft.com/en-us/download/details.aspx?id=48159)
    * [Visual C++ 2015 Redistributable](http://download.microsoft.com/download/5/F/7/5F7ACAEB-8363-451F-9425-68A90F98B238/visualcppbuildtools_full.exe)
    * [Visual Studio Isolated Shell 2015](https://visualstudio.microsoft.com/vs/older-downloads/isolated-shell/)

## Windows
* Build using the following commands
```sh
mkdir <ROOTDIR>/build
cd <ROOTDIR>/build
cmake -DCMAKE_GENERATOR_PLATFORM=x64 ..
cmake --build ./ --config Release
```
* Run the Application
```sh
 cd ROOTDIR>/build/bin/
 ./ccb_parser.exe <optional_ccb_filename>
```

## Linux/Mac
* Build using the following commands
```sh
mkdir <ROOTDIR>/build
cd <ROOTDIR>/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
* Run the Application
```sh
 cd ROOTDIR>/build/bin/
 ./ccb_parser <optional_ccb_filename>
```

# Structure of a CCB file

There are different types of calibration blocks, each with a different ID. Some calibration block ID don't seem to have an associated calibration block. 

```
BLOCK ID Definitions
===================================
CAL_BLOCK_ID_ADDRVAL_REGLIST  = 'A'
CAL_BLOCK_ID_DEVICE_CONFIG    = 'C'
CAL_BLOCK_ID_PIXEL_DEFECT     = 'D'
CAL_BLOCK_ID_FOI_MASK         = 'F'
CAL_BLOCK_ID_GEOMETRIC        = 'G'
CAL_BLOCK_ID_ILLUM_PROFILE    = 'I'
CAL_BLOCK_ID_LSDACS           = 'L'
CAL_BLOCK_ID_P0               = 'P'
CAL_BLOCK_ID_REGISTRATION     = 'R'
CAL_BLOCK_ID_SPATIAL_TEMPCOR  = 'S'
CAL_BLOCK_ID_TEMPCORRECTION   = 'T'
CAL_BLOCK_ID_RELATIVE_ILLUM   = 'V'
CAL_BLOCK_ID_COLUMN_DEFECT    = '|'
CAL_BLOCK_ID_GAINCORRECTION   = 0xD7
CAL_BLOCK_ID_HDR              = 0xFF
```

Each `CAL_BLOCK_ID` has an associated calibration block. The mapping is as follows:

```
CAL_BLOCK_ID_ADDRVAL_REGLIST -> CAL_ADDRVAL_REG_BLOCK_V1
CAL_BLOCK_ID_DEVICE_CONFIG    -> ??? (not used in current .CCB)
CAL_BLOCK_ID_PIXEL_DEFECT     -> ??? (not used in current .CCB)
CAL_BLOCK_ID_FOI_MASK         -> CAL_FOI_MASK_BLOCK_V0_V1INFO
CAL_BLOCK_ID_GEOMETRIC        -> CAL_GEOMETRIC_BLOCK_V3
CAL_BLOCK_ID_ILLUM_PROFILE    -> CAL_ILLUM_PROFILE_BLOCK_V2
CAL_BLOCK_ID_LSDACS           -> CAL_LSDAC_BLOCK_V1
CAL_BLOCK_ID_P0               -> CAL_P0BLOCK_V4 
CAL_BLOCK_ID_REGISTRATION     -> ??? (not used in current .CCB)
CAL_BLOCK_ID_SPATIAL_TEMPCOR  -> CAL_SPATIAL_TEMP_CORR_BLOCK
CAL_BLOCK_ID_TEMPCORRECTION   -> CAL_TEMP_CORR_BLOCK_V0_V1INFO
CAL_BLOCK_ID_RELATIVE_ILLUM   -> CAL_RELATIVE_ILLUM_BLOCK
CAL_BLOCK_ID_COLUMN_DEFECT    -> CAL_COLUMN_DEFECT_BLOCK_V1 (not used in current . CCB)
CAL_BLOCK_ID_GAINCORRECTION   -> CAL_GAIN_CORRECTION_BLOCK
CAL_BLOCK_ID_HDR              -> CAL_HEADER_BLOCK_V3
```

# Parser implementation

The CCB parser is written in C. It goes through a CCB file and writes block by block and cast each block to the write calibration block structure.

Since there can be multiple blocks of the same type in a CCB file, the blocks are stored in a linked list in the `camera_cal` structure. Each field of the `camera_cal` structure is a pointer to the head of the linked list. As a result, the head of a linked list corresponding to a particular calibration block corresponds to the last calibration block of that type read in the CCB file. 

# Comparison to Matlab

The repository contains an example CCB file as well as a `.mat` file that contains a Matlab structure for that CCB file. The `.mat` file can be used to check whether the implementation of parser is correct and the right values are read.

# Future work

* implement some kind of index table to store the calibration blocks with the `camera_cal` structure instead of the linked list
* write helper function to read/write/manipulate/print/... CCB files
* write Python wrapper
* add a CMake file
* write a better Readme