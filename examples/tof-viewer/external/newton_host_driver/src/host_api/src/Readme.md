## Installation

The following third party software packages are used by the voiceui_control program:

* json_parser
* libcutils (the "Android Open Source Project” libcutils package)
* wiringPi (required if using EVENT interrupts on the Raspberry Pi)

### Step 1: Install packages
Downloading and building json_parser and libcutils
* cd /home/$USER/adadn8080/host_api/examples/c
* make install_depends

### Step 2: Build sources

Building voiceui_control with polling for EVENTs
* cd /home/$USER/adadn8080/host_api/examples/c
* make

OR

Building voiceui_control with EVENT interrupts
* cd /home/$USER/adadn8080/host_api/examples/c
* make POLLING=0

### Step 3: Run

Running voiceui_control with EVENT interrupts
* sudo ./linux/voiceui_ctrl_main.exe

Building voiceui_control with polling for EVENTs
* ./linux/voiceui_ctrl_main.exe

## Usage
    voiceui_control.py load firmware_file_name data  
    voiceui_control.py write address data  
    voiceui_control.py read address  
    voiceui_control.py set param_name param_value  
    voiceui_control.py get param_name  

## Manual Install
* Installation of the json_parser
 * The json_parser can be found at https://travis-ci.org/udp/json-parser
 * To clone the git repository:
    * cd /home/$USER
    * git clone https://github.com/udp/json-builder json-builder
 * To build and install:
    * cd /home/$USER/json-parser
    * mkdir -p /home/$USER/json-parser-install/lib
    * mkdir -p /home/$USER/json-parser-install/include
    * ./configure -prefix=/home/$USER/json-parser-install
    * make
    * make install-static

* Installing libcutils
 * Download the following gzipped tar file and unpack into /home/$USER/libcutils
    * https://android.googlesource.com/platform/system/core/+archive/master/libcutils.tar.gz
 * There is no need to build this package. The voiceui_control Makefile builds the required files.

* Installating liblog
 * Download the following gzipped tar file and unpack into /home/$USER/libcutils
    * https://android.googlesource.com/platform/system/core/+archive/master/liblog.tar.gz
 * There is no need to build this package. The voiceui_control Makefile builds the required files.  
   (The libcutils package requires log.h even though no logging is performed.)

### Installing wiringPi
 * If you are using RaspberryPi for your evaluation purposes then you can use WiringPI to use the GPIO functionality to interrupt the host from a ADADN8080 Event. The default of the API implementation uses polling for the ADADN8080 event status  
 * wiringPi can be found at https://projects.drogon.net
 * Download the following gzipped tar file and unpack into /home/$USER/wiringPi
    * https://git.drogon.net/?p=wiringPi;a=snapshot;h=HEAD;sf=tgz
 * To build and install:
    * cd /home/$USER
    * gunzip wiringPi-HEAD-xxx.tar.gz
    * tar xf wiringPi-HEAD-xxx.tar
    * mv wiringPi-HEAD-xxx wiringPi
    * ./build


