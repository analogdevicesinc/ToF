/****************************************************************************
* Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
*******************************************************************************
*******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.*/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "../crc32/crc.h"

#include <iostream>
#include <vector>
#include <fstream>
#include "../include/adsd3500_util.h"

const char ver_info[] =
    "VERSIONINFO:"
    "TOF_DepthComputeEngine_ARM64-Rel4.4.0";

#define ADSD3500_ADDR (0x70 >> 1)
#define ADSD3500_INTRINSIC_SIZE		56
#define ADSD3500_DEALIAS_SIZE		32
#define ADI_STATUS_FIRMWARE_UPDATE      0x000E
#define ADI_STATUS_NVM_WRITE_COMPLETE 0x000F
#define BUF_SIZE (4000000)
#define PAGE_SIZE 256u
#define CTRL_SIZE 4099
#define IOCTL_TRIES 3
#define FULL_UPDATE_CMD 0x12
#define MAX_BIN_SIZE	741376

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
#define ADSD3500_ADDR (0x70 >> 1)
#define DEBUG 1

Adsd3500::Adsd3500() {
    // Do Nothing.
}

Adsd3500::~Adsd3500() {
    // Stop the Stream.
    if (videoDevice.started) {
        StopStream();
    }

    // Close the Camera.
    for (unsigned int i = 0; i < videoDevice.nVideoBuffers; i++) {
        if (munmap(videoDevice.videoBuffers[i].start,
                   videoDevice.videoBuffers[i].length) == -1) {
            std::cout
                << "munmap error "
                << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        }
    }

    free(videoDevice.videoBuffers);

    if (videoDevice.videoCaptureDeviceId != -1) {
        if (close(videoDevice.videoCaptureDeviceId) == -1) {
            std::cout << "Unable to close the video capture device." << std::endl;
        }
    }

    if (videoDevice.cameraSensorDeviceId != -1) {
        if (close(videoDevice.cameraSensorDeviceId) == -1) {
            std::cout << "Unable to close the camera sensor device." << std::endl;
        }
    }
}

/*
*****************************ADSD3500 Control Commands****************************
*/

// Reset.
uint8_t reset_cmd[] = {0x00, 0x24, 0x00, 0x00};

// Set Image Mode.
/*
Set the second byte of the 'setMode_cmd[]' with the desired Imaging Mode. (Available modes 0 to 4)
For eg. to set Mode 1 as the imaging mode, the Mode byte value should be set like this 'setMode_cmd[1] = 0x01'. 
*/
uint8_t setMode_cmd[] = {0xDA, 0x00, 0x28, 0x0F}; // Mode 3 lr-qnative 

// Get Image Mode.
uint8_t getMode_cmd[] = {0x00, 0x12};

// Stream on.
uint8_t streamOn_cmd[] = {0x00, 0xAD, 0x00, 0xC5};

// Stream off.
uint8_t streamOff_cmd[] = {0x00, 0x0C, 0x00, 0x02};

// Set FPS value.
/*
To set the FPS value in the ADSD3500, set the fourth byte of the 'setFps_cmd[]' with the desired FPS value.
For eg. to set the FPS as 20, the setFps_cmd[3] should be assigned the value 0x14.
*/
uint8_t setFps_cmd[] = {0x00, 0x22, 0x00, 0x00};

// Get FPS value set in the ADSD3500.
uint8_t getFps_cmd[] = {0x00, 0x23};

// Switch to Burst Mode command.
uint8_t switchToBurstMode_cmd[] = {0x00, 0x19, 0x00, 0x00};

// Switch to Standard Mode command.
uint8_t switchToStandardMode_cmd[] = 
    {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

// Get Dealias Parameters command.
uint8_t getDealiasParams_cmd[] = {0xAD, 0x00, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, \
		0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Get Camera Intrinsics command.
uint8_t getCameraIntrinsics_cmd[] = {0xAD, 0x00, 0x38, 0x01, 0x00, 0x00, 0x00, 0x00, \
		0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Read Chip ID
uint8_t getChipId_cmd[] = {0x01, 0x12};

// Get Imager Type and CCB version
uint8_t getImagerTypeAndCCB_cmd[] = {0x00, 0x32};

/*
*********************************Public functions*********************************
*/ 

// Resets ADSD3500 device.
int Adsd3500::ResetAdsd3500() {
	printf("Resetting ADSD3500 Device.\n");
    system("echo 0 > /sys/class/gpio/gpio122/value");
    usleep(1000000);
    system("echo 1 > /sys/class/gpio/gpio122/value");
    usleep(7000000);

	return 0;
}

// Opens ADSD3500 device.
int Adsd3500::OpenAdsd3500() {
	// Open the ADI ToF Camera Sensor Device Driver.
	videoDevice.cameraSensorDeviceId = tof_open(CAMERA_SENSOR_DRIVER);
    if (videoDevice.cameraSensorDeviceId < 0) {
		std::cout << "Unable to open camera sensor device: " << CAMERA_SENSOR_DRIVER << std::endl;
        return -1;
	}
    
    // Open Host device's V4L2 Video Capture Device Driver.
    videoDevice.videoCaptureDeviceId = tof_open(VIDEO_CAPTURE_DRIVER);
    if (videoDevice.videoCaptureDeviceId < 0) {
		std::cout << "Unable to open video capture device:  " << VIDEO_CAPTURE_DRIVER << std::endl;
        return -1;
    }

	return 0;
}

// Sets Imaging mode.
int Adsd3500::SetImageMode(uint8_t modeNumber) {
	setMode_cmd[1] = modeNumber;

	printf("Setting Imaging mode number as %d.\n", modeNumber);

	int32_t ret = write_cmd(videoDevice.cameraSensorDeviceId, setMode_cmd, ARRAY_SIZE(setMode_cmd));
	std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;

    static struct v4l2_control ctrl;

    memset(&ctrl, 0, sizeof(ctrl));

    ctrl.id = CTRL_SET_MODE;
    ctrl.value = modeNumber;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cout << "Setting Mode error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

	return ret;
}

// Gets Imaging mode.
int Adsd3500::GetImageMode(uint8_t* result) {

	int32_t ret = read_cmd(videoDevice.cameraSensorDeviceId, getMode_cmd, ARRAY_SIZE(getMode_cmd), result, ARRAY_SIZE(result));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
	return ret;
}

// Starts the stream.
int Adsd3500::StartStream() {
    struct v4l2_buffer buf;

    // Set Toggle mode
    int ret = adsd3500_write_cmd(0x0025, 1); /* Mode = 1 adsd3500 fsync automatically toggles at user specified framerate*/
    if (ret < 0) {
        perror("Unable to set Toggle mode.");
        return -1;
    }

	printf("Starting the stream.\n");
    std::cout << "Number of video buffers: " << videoDevice.nVideoBuffers << std::endl;

    for (unsigned int i = 0; i < videoDevice.nVideoBuffers; i++) {
        CLEAR(buf);
        buf.type = videoDevice.videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = videoDevice.planes;
        buf.length = 1;

        if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_QBUF, &buf) == -1) {
            std::cout
                << "mmap error "
                << "errno: " << errno << " error: " << strerror(errno) << std::endl;
            return -1;
        }
    }

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_STREAMON, &videoDevice.videoBuffersType) != 0) {
        std::cout << "VIDIOC_STREAMON error "
                         << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    videoDevice.started = true;

	return 0;
}

// Stops the stream.
int Adsd3500::StopStream() {
	printf("Stopping the stream.\n");
	int32_t ret = write_cmd(videoDevice.cameraSensorDeviceId, streamOff_cmd, ARRAY_SIZE(streamOff_cmd));
	std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    videoDevice.started = false;
	return ret;
}

// Get Frames from the Imager.
int Adsd3500::RequestFrame(uint16_t* buffer) {
    struct v4l2_buffer buf[MAX_SUBFRAMES_COUNT];
    unsigned int buf_data_len;
    uint8_t *pdata;
    int ret = 0;

    ret = adsd3500_wait_for_buffer();
    if (ret < 0) {
        return -1;
    }

    ret = adsd3500_dequeue_internal_buffer(buf[0]);
    if (ret < 0) {
        return -1;
    }

    ret = adsd3500_get_internal_buffer(&pdata, buf_data_len, buf[0]);
    if (ret < 0) {
        return -1;
    }

    if (pdata == NULL) {
        printf("No frame received.\n");
        return -1;
    }

    memcpy(buffer, pdata, buf_data_len);

    ret = adsd3500_enqueue_internal_buffer(buf[0]);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

// Reads the .ini file and stores them as key-value pairs.
int Adsd3500::GetIniKeyValuePairFromConfig(const char* iniFileName) {

    int ret = 0;
    ret = adsd3500_get_key_value_pairs_from_ini(iniFileName, iniKeyValPairs);
    if (ret < 0) {
        perror("Unable to get Ini Key Value pairs from the .ini file");
    }

    return ret;
}

// Configure Adsd3500 with parameters from .ini file.
int Adsd3500::ConfigureAdsd3500WithIniParams() {

    if (iniKeyValPairs.empty()) {
        perror("Key-value pairs from the .ini file not read.\n");
        return -1;
    }
    int ret = adsd3500_set_ini_params(iniKeyValPairs);
    if (ret < 0) {
        perror("Unable to set Ini parameters in Adsd3500.\n");
    }

    return ret;
}

// Configure Depth Compute Library with parameters from .ini file.
int Adsd3500::ConfigureDepthComputeLibraryWithIniParams() {
    
    uint32_t ret = 0;
    std::cout << "Initializing Depth Compute Library." << std::endl;

    std::cout << "Ini File Size: " << iniFileData.size << std::endl;
    if (iniFileData.p_data == nullptr) {
        perror("iniFileData is NULL.\n");
        return -1;
    }

    tofi_config = InitTofiConfig_isp(&iniFileData, 0, &ret, &xyzDealiasData);
    if (tofi_config == NULL) {
        perror("InitTofiConfig failed.\n");
        return -1;
    }

    tofi_compute_context = 
        InitTofiCompute(tofi_config->p_tofi_cal_config, &ret);
    if (tofi_compute_context == NULL) {
        perror("InitTofiCompute failed.\n");
        return -1;
    }

    return 0;
}

// Configures V4L2 MIPI Capture Driver and Camera Sensor Driver.
int Adsd3500::ConfigureDeviceDrivers() {  
    int ret = 0;  
    struct v4l2_capability cap;
    const char* expectedCaptureDeviceName = CAPTURE_DEVICE_NAME;

    // Check if the video capture device is valid.
    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP error.\n");
        return -1;
    }

    if (strncmp((char *)cap.card, expectedCaptureDeviceName, strlen(expectedCaptureDeviceName))) {
        perror("Invalid Capture Device name read.\n");
        return -1;
    }

    if (!(cap.capabilities &
        (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))
    ) {
        perror("The device is not a video capture device.\n");
        return -1;
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        videoDevice.videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        printf("Video buffers type: V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE.\n");
    } else {
        videoDevice.videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("Video buffers type: V4L2_BUF_TYPE_VIDEO_CAPTURE.\n");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        perror("The device does not support streaming.\n");
        return -1;
    }

    // Check if the camera sensor driver is valid.
    uint8_t chip_id_value[2] = {0x00, 0x00};
    ret = Adsd3500::ReadChipId(chip_id_value);
    if (ret < 0){
        perror("Unable to get the Chip ID.\n");
        return -1;
    } else {
        PrintByteArray(chip_id_value, ARRAY_SIZE(chip_id_value));
    }

    return 0;
}

// Gets Camera Intrinsics and Dealias Parameters from ADSD3500.
int Adsd3500::GetIntrinsicsAndDealiasParams() {
    int ret = 0;

    uint8_t intrinsics[56] = {0};
    uint8_t dealiasParams[32] = {0};

    intrinsics[0] = mode_num;
    dealiasParams[0] = mode_num;

    ret = adsd3500_read_payload_cmd(0x01, intrinsics, 56);
    if (ret < 0) {
        perror("Unable to get intrinsics.\n");
    }

    ret = adsd3500_read_payload_cmd(0x02, dealiasParams, 32);
    if (ret < 0) {
        perror("Unable to get dealias params.\n");
    }

    memcpy(&xyzDealiasData, dealiasParams,
            sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));
    memcpy(&xyzDealiasData.camera_intrinsics, intrinsics,
            sizeof(CameraIntrinsics));

    return 0;
}

// Parses Raw frame to get Depth, AB and Confidence frames using Depth-Compute Library.
int Adsd3500::ParseRawDataWithDCL(uint16_t* buffer) {

    int enableXyz = 0;
    auto it = iniKeyValPairs.find("xyzEnable");
    if (it != iniKeyValPairs.end()) {
        enableXyz = (std::stoi(it->second));
    }

    if (tofi_compute_context == NULL) {  
        return -1;
    }

    uint16_t *tempDepthFrame = tofi_compute_context->p_depth_frame;
    uint16_t *tempAbFrame = tofi_compute_context->p_ab_frame;
    //uint16_t *tempXyzFrame = (uint16_t *)tofi_compute_context->p_xyz_frame;

    // Allocate memory to store Depth and IR frames.
    // Depth Frames. 
    tofi_compute_context->p_depth_frame = 
        new uint16_t[xyzDealiasData.n_rows*xyzDealiasData.n_cols];
    // IR Frames.
    tofi_compute_context->p_ab_frame = 
        new uint16_t[xyzDealiasData.n_rows*xyzDealiasData.n_cols];

    uint32_t ret =
            TofiCompute(buffer, tofi_compute_context, NULL);
    if (ret < 0) {
        perror("Failed to parse frames with Depth Compute Library.\n");
    }

    if (tofi_compute_context->p_ab_frame == NULL) {
        perror("Error in retrieving AB frame.\n");
        return -1;
    }

    if (tofi_compute_context->p_depth_frame == NULL) {
        perror("Error in retrieving Depth frame.\n");
        return -1;
    }

    if (tofi_compute_context->p_conf_frame == NULL) {
        perror("Error in retrieving Confidence frame.\n");
        return -1;
    }
    
    return 0;
}

// Read ADSD3500 Status
int Adsd3500::ReadChipId(uint8_t* result) {
    int32_t fd = 0;

    if (videoDevice.cameraSensorDeviceId == -1) {
        fd = tof_open(CAMERA_SENSOR_DRIVER);
        if (fd < 0) {
		    std::cout << "Unable to find camera: " << CAMERA_SENSOR_DRIVER << std::endl;
            return fd;
	    }
    } else {
        fd = videoDevice.cameraSensorDeviceId;
    }

    int32_t ret = read_cmd(fd, getChipId_cmd, ARRAY_SIZE(getChipId_cmd), result, ARRAY_SIZE(result));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;

    return ret;
}

// Set FPS value.
int Adsd3500::SetFps(int fps) {
    int ret = 0;

// #ifdef NVIDIA
//     struct v4l2_ext_control extCtrl;
//     struct v4l2_ext_controls extCtrls;
//     memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
//     memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));
//     extCtrls.count = 1;
//     extCtrls.controls = &extCtrl;
//     extCtrl.id = CTRL_SET_FRAME_RATE;
//     extCtrl.value = fps;
//     if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
//         std::cout << "Failed to set FPS.  "
//                      << "errno: " << errno << " error: " << strerror(errno) << std::endl;
//         return -1;
//     }
// #else // NXP
    struct v4l2_streamparm fpsControl;
    memset(&fpsControl, 0, sizeof(struct v4l2_streamparm));

    fpsControl.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fpsControl.parm.capture.timeperframe.numerator = 1;
    fpsControl.parm.capture.timeperframe.denominator = fps;

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_S_PARM, &fpsControl) == -1) {
        std::cout << "Failed to set FPS."
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }
//#endif

    ret = this->adsd3500_write_cmd(0x22, fps);
    if (ret < 0) {
        std::cout << "Failed to set fps at: " << fps
                   << "via host commands!" << std::endl;
        return -1;
    }

    return ret;

    // int32_t fd = 0;

    // if (videoDevice.cameraSensorDeviceId == -1) {
    //     int32_t fd = tof_open(CAMERA_SENSOR_DRIVER);
    //     if (fd < 0) {
	// 	    std::cout << "Unable to find camera: " << CAMERA_SENSOR_DRIVER << std::endl;
    //         return fd;
	//     }
    // } else {
    //     fd = videoDevice.cameraSensorDeviceId;
    // }

    // printf("Setting the desired FPS value.\n");

    // setFps_cmd[3] = fps;
    // int32_t ret = write_cmd(fd, setFps_cmd, ARRAY_SIZE(setFps_cmd));
    // std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;

    // return ret;
}

// Get FPS value.
int Adsd3500::GetFps(uint8_t* result) {
    int32_t fd = 0;

    if (videoDevice.cameraSensorDeviceId == -1) {
        int32_t fd = tof_open(CAMERA_SENSOR_DRIVER);
        if (fd < 0) {
		    std::cout << "Unable to find camera: " << CAMERA_SENSOR_DRIVER << std::endl;
            return fd;
	    }
    } else {
        fd = videoDevice.cameraSensorDeviceId;
    }
    int32_t ret = read_cmd(fd, getFps_cmd, ARRAY_SIZE(getFps_cmd), result, ARRAY_SIZE(result));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;

    return ret;
}

// Read CCB from ADSD3500.
int Adsd3500::ReadCCB(const char* filename) {

    uint8_t data[CTRL_SIZE] = {0};
	char binbuff[BUF_SIZE] = {0};
	uint32_t chunkSize = 2048u;
	uint32_t sizeOfBinary = 0u;
	uint32_t checksum = 0u;
	uint32_t packetsNeeded = 0u;

    int ret;
    // Open the ToF Camera device
	int32_t fd = tof_open(CAMERA_SENSOR_DRIVER);
	if (fd < 0) {
		std::cout << "Unable to find camera: " << CAMERA_SENSOR_DRIVER << std::endl;
        return fd;
	}

    std::ofstream fp(filename, std::ofstream::binary);
	if (!fp) {
		std::cout<<"Cannot open file!"<<std::endl;
		return 1;
	}
	
	//Find the binary file size
	fp.seekp(0, fp.beg);

    // Switch to Burst Mode.
    ret =  adsd3500_switch_to_burst_mode(fd);
    if (ret < 0) {
        printf("Unable to switch to Burst mode. Exiting..\n");
        return ret;
    }

    //Write header and readback response
	memset(data, 0x00, 16);
	data[0] = 1; //WRITE
	data[2] = 16;
	
	data[3] = 0xAD;
	data[6] = 0x13;
	data[11] = 0x13;
	data[15] = 1;
	v4l2_ctrl_set(fd, 0x009819e1, data);

	usleep(1000 * 1);	

	data[0] = 0;
	data[2] = 16;
	v4l2_ctrl_set(fd, 0x009819e1, data);

	v4l2_ctrl_get(fd, 0x009819e1, data);
	
	std::cout<<"Done reading response header "<<std::endl;

	chunkSize = (data[5] << 8) | data[4];
	sizeOfBinary = (data[10] << 24) | (data[9] << 16) | (data[8] << 8) | data[7];

	for (uint32_t i=3; i<11; i++)
		checksum += data[i];

	packetsNeeded = sizeOfBinary / chunkSize;
	std::cout<<"Size of Binary :: "<<sizeOfBinary<<std::endl;
	std::cout<<"ChunkSize :: "<<chunkSize<<std::endl;
	std::cout<<"Packets Needed: "<<packetsNeeded + 1<<std::endl;

	for (uint8_t i=0; i<19; i++) {
		printf("0x%.2X\n", data[i]);
	}

	for (uint32_t i=1; i<=packetsNeeded; i++) {
		data[0] = 0;
		data[1] = chunkSize >> 8;
		data[2] = chunkSize & 0xFF;
		
		usleep(1000 * 30);

		bool retval = v4l2_ctrl_set(fd, 0x009819e1, data);
		v4l2_ctrl_get(fd, 0x009819e1, data);
		if (retval == false) {
			return -1;
		}
		memcpy(binbuff, &data[3], chunkSize);
		fp.write(binbuff, chunkSize);

		std::cout<<"Packet number : "<<i<<" / "<<packetsNeeded + 1<<std::endl;
	}

	chunkSize = sizeOfBinary % chunkSize;
	usleep(1000 * 30);
	data[0] = 0;
	data[1] = chunkSize >> 8;
	data[2] = chunkSize & 0xFF;
		
	v4l2_ctrl_set(fd, 0x009819e1, data);
	v4l2_ctrl_get(fd, 0x009819e1, data);
	memcpy(binbuff, &data[3], chunkSize);
	fp.write(binbuff, chunkSize - 4);
	std::cout<<"Packet number : "<<packetsNeeded + 1<<" / "<<packetsNeeded + 1<<std::endl;

	std::cout<<"Binary size : "<<sizeOfBinary<<std::endl;
	std::cout<<std::endl;

    // Switch to Standard Mode.
    ret =  adsd3500_switch_to_standard_mode(fd);
    if (ret < 0) {
        printf("Unable to switch to Standard mode. Exiting..\n");
        return ret;
    }

    fp.close();

    return 0;
}

// Gets Imager Type and CCB information.
int Adsd3500::GetImagerTypeAndCCB() {

    uint8_t result[2] = {0x00, 0x00};

    int32_t ret = read_cmd(videoDevice.cameraSensorDeviceId, getImagerTypeAndCCB_cmd, ARRAY_SIZE(getImagerTypeAndCCB_cmd), result, ARRAY_SIZE(result));
    if (ret < 0) {
        return -1;
    }

    uint8_t imager_version = result[0];
    uint8_t ccb_version = result[1];

    switch(imager_version){
        case 1: {
            imagerType = ImagerType::IMAGER_ADSD3100;
            printf("Imager type is  IMAGER_ADSD3100.\n");
            break;
        }
        case 2: {
            imagerType = ImagerType::IMAGER_ADSD3030;
            printf("Imager type is  IMAGER_ADSD3030.\n");
            break;
        }
        default: {
            imagerType = ImagerType::IMAGER_UNKNOWN;
            std::cout << "Unsupported Imager found." << std::endl; 
        }
    }

    switch(ccb_version){
        case 1: {
            ccbVersion = CCBVersion::CCB_VERSION0;
            printf("CCB Version is  CCB_VERSION0.\n");
            break;
        }
        case 2: {
            ccbVersion = CCBVersion::CCB_VERSION1;
            printf("CCB version is  CCB_VERSION1.\n");
            break;
        }
        default: {
            ccbVersion = CCBVersion::CCB_UNKNOWN;
            std::cout << "Unsupported CCB version found." << std::endl; 
        }
    }

    return ret;
}

/*
********************************Private functions*********************************
*/ 

int Adsd3500::adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                                 unsigned int usDelay) {

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 2;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return -1;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    usleep(usDelay);

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")" << std::endl;
        return -1;
    }

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")" << std::endl;
        return -1;
    }

    *data = (uint16_t)(extCtrl.p_u8[3] << 8) + (uint16_t)(extCtrl.p_u8[4]);

    return 0;
}

int Adsd3500::adsd3500_write_cmd(uint16_t cmd, uint16_t data) {

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 4;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    buf[5] = uint8_t(data >> 8);
    buf[6] = uint8_t(data & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")" << std::endl;
        return -1;
    }
    return 0;
}


int Adsd3500::adsd3500_read_payload_cmd(uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    int ret = adsd3500_write_cmd(switchCmd, switchPayload);
    if (ret < 0) {
        std::cout << "Failed to switch to burst mode!";
        return -1;
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x01;
    buf[1] = 0x00;
    buf[2] = 0x10;

    buf[3] = 0xAD;
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, readback_data, 1);
    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Reading Adsd3500 error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    if (cmd == 0x13) {
        usleep(1000);
    }

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Reading Adsd3500 error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to get ctrl with id " << extCtrl.id << std::endl;
        return -1;
    }

    memcpy(readback_data, extCtrl.p_u8 + 3, payload_len);

    //If we use the read ccb command we need to keep adsd3500 in burst mode
    if (cmd == 0x13) {
        return 0;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Switch Adsd3500 to standard mode error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int Adsd3500::adsd3500_read_payload(uint8_t *payload, uint16_t payload_len) {

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    usleep(30000);

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Reading Adsd3500 error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to get ctrl with id " << extCtrl.id  << std::endl;
        return -1;
    }

    memcpy(payload, extCtrl.p_u8 + 3, payload_len);

    return 0;
}

int Adsd3500::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload, uint16_t payload_len) {

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    int ret = adsd3500_write_cmd(switchCmd, switchPayload);
    if (ret < 0) {
        std::cout << "Failed to switch to burst mode!" << std::endl;
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    payload_len += 16;
    buf[0] = 0x01;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    payload_len -= 16;
    buf[3] = 0xAD;
    buf[4] = uint8_t(payload_len >> 8);
    buf[5] = uint8_t(payload_len & 0xFF);
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Writing Adsd3500 error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Switch Adsd3500 to standard mode error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int Adsd3500::adsd3500_write_payload(uint8_t *payload, uint16_t payload_len) {

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    memcpy(buf + 3, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Writing Adsd3500 error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    usleep(100000);

    return 0;
}

int Adsd3500::adsd3500_switch_to_burst_mode(int fd) {
    
    printf("Switching to Burst mode.\n");
    int32_t ret = write_cmd(fd, switchToBurstMode_cmd, ARRAY_SIZE(switchToBurstMode_cmd));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    return ret;
    
}

int Adsd3500::adsd3500_switch_to_standard_mode(int fd) {
    
    printf("Switching to Standard mode.\n");
    int32_t ret = write_cmd(fd, switchToStandardMode_cmd, ARRAY_SIZE(switchToStandardMode_cmd));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    return ret;
}


int Adsd3500::adsd3500_wait_for_buffer() {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(videoDevice.videoCaptureDeviceId, &fds);

    tv.tv_sec = 20;
    tv.tv_usec = 0;

    r = select(videoDevice.videoCaptureDeviceId + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        std::cout << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return -1;
    } else if (r == 0) {
        std::cout << "select timeout";
        return -1;
    }
    return 0;
}

int Adsd3500::adsd3500_dequeue_internal_buffer(struct v4l2_buffer &buf) {

    CLEAR(buf);
    buf.type = videoDevice.videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = videoDevice.planes;

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_DQBUF, &buf) == -1) {
        std::cout << "VIDIOC_DQBUF error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        switch (errno) {
        case EAGAIN:
        case EIO:
            break;
        default:
            return -1;
        }
    }

    if (buf.index >= videoDevice.nVideoBuffers) {
        std::cout << "Not enough buffers avaialable";
        return -1;
    }

    return 0;
}

int Adsd3500::adsd3500_enqueue_internal_buffer(struct v4l2_buffer &buf) {

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_QBUF, &buf) == -1) {
        std::cout << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int Adsd3500::adsd3500_get_internal_buffer(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf
) {

    *buffer = static_cast<uint8_t *>(videoDevice.videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return 0;
}

int Adsd3500::adsd3500_get_key_value_pairs_from_ini(
    const std::string &iniFileName,
    std::map<std::string, std::string> &iniKeyValPairs) {

    std::ifstream iniStream(iniFileName);
    if (!iniStream.is_open()) {
        std::cout << "Failed to open: " << iniFileName << std::endl;
        return -1;
    }

    // Get the size of the file
    iniStream.seekg(0, std::ios::end);
    std::streampos fileSize = iniStream.tellg();
    iniStream.seekg(0, std::ios::beg);
    
    // Check if the fileSize is representable as size_t
    if (fileSize < 0 || static_cast<size_t>(fileSize) != static_cast<std::streampos>(fileSize)) {
        std::cerr << "File size is too large to be represented as size_t." << std::endl;
        return -1;
    }

    iniFileData.size = static_cast<size_t>(fileSize);

    // Allocate memory for the unsigned char array
    iniFileData.p_data = new unsigned char[fileSize];

    // Read the contents of the file into the buffer
    iniStream.read(reinterpret_cast<char*>(iniFileData.p_data), fileSize);

    // Going to the start of the file to read it's contents.
    iniStream.seekg(0, std::ios::beg);

    iniKeyValPairs.clear();

    std::string line;
    while (getline(iniStream, line)) {
        size_t equalPos = line.find('=');
        if (equalPos == std::string::npos) {
            std::cout << "Unexpected format on this line:\n"
                         << line << "\nExpecting 'key=value' format";
            continue;
        }
        std::string key = line.substr(0, equalPos);
        std::string value = line.substr(equalPos + 1);
        if (!value.empty()) {
            iniKeyValPairs.emplace(key, value);
        } else {
            std::cout << "No value found for parameter: " << key;
        }
    }

    iniStream.close();

    return 0;
}

int Adsd3500::adsd3500_set_ini_params(
    const std::map<std::string, std::string> &iniKeyValPairs) {

    auto it = iniKeyValPairs.find("abThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0010, std::stoi(it->second));
    } else {
        std::cout << "abThreshMin was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("confThresh");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0011, std::stoi(it->second));
    } else {
        std::cout << "confThresh was not found in .ini file, not setting." << std::endl;;
    }

    it = iniKeyValPairs.find("radialThreshMin");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0027, std::stoi(it->second));
    } else {
        std::cout
            << "radialThreshMin was not found in .ini file, not setting." << std::endl;;
    }

    it = iniKeyValPairs.find("radialThreshMax");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0029, std::stoi(it->second));
    } else {
        std::cout
            << "radialThreshMax was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("jblfWindowSize");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0014, std::stoi(it->second));
    } else {
        std::cout
            << "jblfWindowSize was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("jblfApplyFlag");
    if (it != iniKeyValPairs.end()) {
        bool en = !(it->second == "0");
        adsd3500_write_cmd(0x0013, en ? 1 : 0);
    } else {
        std::cout
            << "jblfApplyFlag was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("fps");
    if (it != iniKeyValPairs.end()) {
        SetFps(std::stoi(it->second));
    } else {
        std::cout << "fps was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("vcselDelay");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0066, std::stoi(it->second));
    } else {
        std::cout << "vcselDelay was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("jblfMaxEdge");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0074, std::stoi(it->second));
    } else {
        std::cout << "jblfMaxEdge was not found in .ini file, "
                        "not setting." << std::endl;
    }

    it = iniKeyValPairs.find("jblfABThreshold");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0075, std::stoi(it->second));
    } else {
        std::cout << "jblfABThreshold was not found in .ini file" << std::endl;
    }

    it = iniKeyValPairs.find("jblfGaussianSigma");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x006B, std::stoi(it->second));
    } else {
        std::cout
            << "jblfGaussianSigma was not found in .ini file, not setting." << std::endl;
    }

    it = iniKeyValPairs.find("jblfExponentialTerm");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x006C, std::stoi(it->second));
    } else {
        std::cout << "jblfExponentialTerm was not found in .ini file, "
                        "not setting." << std::endl;
    }

    it = iniKeyValPairs.find("enablePhaseInvalidation");
    if (it != iniKeyValPairs.end()) {
        adsd3500_write_cmd(0x0072, std::stoi(it->second));
    } else {
        std::cout << "enablePhaseInvalidation was not found in .ini file, "
                        "not setting." << std::endl;
    }

    return 0;
}

int Adsd3500::SetFrameType() {

    int ret = 0;

    if (videoDevice.started) {
        StopStream();
    }    

    for (unsigned int i = 0; i < videoDevice.nVideoBuffers; i++) {
        if (munmap(videoDevice.videoBuffers[i].start,
                   videoDevice.videoBuffers[i].length) == -1) {
            std::cout
                << "munmap error "
                << "errno: " << errno << " error: " << strerror(errno);
        }
    }
    free(videoDevice.videoBuffers);

    if (videoDevice.videoCaptureDeviceId != -1) {
        if (close(videoDevice.videoCaptureDeviceId) == -1) {
            std::cout << "Unable to close V4L2 Capture Device driver" << std::endl;
        }
    }

    if (videoDevice.cameraSensorDeviceId != -1) {
        if (close(videoDevice.cameraSensorDeviceId) == -1) {
            std::cout << "Unable to close V4L2 Camera Sensor driver" << std::endl;
        }
    }

    ret = OpenAdsd3500();
    if (ret < 0) printf("Unable to Open Adsd3500.\n");

    struct v4l2_requestbuffers req;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;

    ret = SetImageMode(mode_num);
    if (ret < 0)  printf("Unable to Set mode in Adsd3500.\n");

    CLEAR(req);
    req.count = 0;
    req.type = videoDevice.videoBuffersType;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_REQBUFS, &req) == -1) {
        std::cout
            << "VIDIOC_REQBUFS error "
            << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    std::cout << "Values from INI file" << std::endl;

    auto it = iniKeyValPairs.find("bitsInPhaseOrDepth");
    if (it != iniKeyValPairs.end()) {
        depthBits = std::stoi(it->second);
        std::cout << "Number of depth bits : " << depthBits << std::endl;
    }
    it = iniKeyValPairs.find("bitsInAB");
    if (it != iniKeyValPairs.end()) {
        abBits = std::stoi(it->second);
        std::cout << "Number of AB bits : " << abBits << std::endl;
    }
    it = iniKeyValPairs.find("bitsInConf");
    if (it != iniKeyValPairs.end()) {
        confBits = std::stoi(it->second);
        std::cout << "Number of Confidence bits: " << confBits << std::endl;
    }

    adsd3500_configure_sensor_frame_types();

    // Pixel format is "raw8" for lr-qnative mode.
    int frameHeight = xyzDealiasData.n_rows;
    int frameWidth = xyzDealiasData.n_cols;
    float totalBits = depthBits + abBits + confBits;
    uint16_t width = frameWidth * totalBits / 8;
    uint16_t height = frameHeight;
    __u32 pixelFormat = V4L2_PIX_FMT_SBGGR8; // if raw8 format, use this as pixel format.

    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    // Set the frame format in the driver.
    CLEAR(fmt);
    fmt.type = videoDevice.videoBuffersType;
    fmt.fmt.pix.pixelformat = pixelFormat;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_S_FMT, &fmt) == -1) {
        std::cout << "Setting Pixel Format error, errno: " << errno
                << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    // Allocate the video buffers in the driver.
    CLEAR(req);
    req.count = 4; // (m_capturesPerFrame + EXTRA_BUFFERS_COUNT)
    req.type = videoDevice.videoBuffersType;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_REQBUFS, &req) == -1) {
        std::cout
        << "VIDIOC_REQBUFS error "
        << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    videoDevice.videoBuffers = (buffer *)calloc(req.count, sizeof(*videoDevice.videoBuffers));
    if (!videoDevice.videoBuffers) {
        std::cout << "Unable to allocate video buffers in the driver" << std::endl;
        return -1;
    }

    int length, offset;
    for (videoDevice.nVideoBuffers = 0; videoDevice.nVideoBuffers < req.count;
                 videoDevice.nVideoBuffers++) {

        CLEAR(buf);
        buf.type = videoDevice.videoBuffersType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = videoDevice.nVideoBuffers;
        buf.m.planes = videoDevice.planes;
        buf.length = 1;

        if (xioctl(videoDevice.videoCaptureDeviceId, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cout
            << "VIDIOC_QUERYBUF error "
            << "errno: " << errno << " error: " << strerror(errno) << std::endl;
            return -1;
        }

        if (videoDevice.videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            length = buf.length;
            offset = buf.m.offset;
        } else {
            length = buf.m.planes[0].length;
            offset = buf.m.planes[0].m.mem_offset;
        }

        videoDevice.videoBuffers[videoDevice.nVideoBuffers].start = 
            mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, videoDevice.videoCaptureDeviceId, offset);

        if (videoDevice.videoBuffers[videoDevice.nVideoBuffers].start == MAP_FAILED) {
            std::cout
                << "mmap error "
                << "errno: " << errno << " error: " << strerror(errno) << std::endl;
            return -1;
        }

        videoDevice.videoBuffers[videoDevice.nVideoBuffers].length = length;
    }

    return 0;
}

int Adsd3500::adsd3500_set_control(const std::string &control, const std::string &value) {
    // Set control commands to set value in the V4L2 Camera sensor driver.
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));


    if (control == "bitsInPhaseOrDepth") {
        ctrl.id = CTRL_PHASE_DEPTH_BITS;
    } else if (control == "bitsInAB") {
        ctrl.id = CTRL_AB_BITS;
    } else if (control == "bitsInConf") {
        ctrl.id = CTRL_CONFIDENCE_BITS;
    } else if (control == "depthEnable") {
        ctrl.id = CTRL_DEPTH_EN;
    } else if (control == "abAveraging") {
        ctrl.id = CTRL_AB_AVG;
    } else {
        std::cout << "Invalid Control Command passed: " << control <<  std::endl;
    }

    ctrl.value = std::stoi(value);

    if (xioctl(videoDevice.cameraSensorDeviceId, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cout << "Failed to set control: " << control << " "
                     << "errno: " << errno << " error: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int Adsd3500::adsd3500_configure_sensor_frame_types() {
    std::string value;

    // Configure Phase/Depth bits.
    auto it = iniKeyValPairs.find("bitsInPhaseOrDepth");
    if (it != iniKeyValPairs.end()) {
        value = it->second;
        if (value == "16")
            value = "6";
        else if (value == "14")
            value = "5";
        else if (value == "12")
            value = "4";
        else if (value == "10")
            value = "3";
        else
            value = "2";
        adsd3500_set_control("bitsInPhaseOrDepth", value);
    } else {
        std::cout <<  "bitsInPhaseOrDepth was not found in .ini file" << std::endl;
    }

    // Configure Confidence bits.
    it = iniKeyValPairs.find("bitsInConf");
    if (it != iniKeyValPairs.end()) {
        value = it->second;
        if (value == "8")
            value = "2";
        else if (value == "4")
            value = "1";
        else
            value = "0";
        adsd3500_set_control("bitsInConf", value);
    } else {
        std::cout << "bitsInConf was not found in .ini file" << std::endl;
    }

    // Configure AB bits.
    it = iniKeyValPairs.find("bitsInAB");
    if (it != iniKeyValPairs.end()) {
        value = it->second;
        if (value == "16")
            value = "6";
        else if (value == "14")
            value = "5";
        else if (value == "12")
            value = "4";
        else if (value == "10")
            value = "3";
        else if (value == "8")
            value = "2";
        else {
            value = "0";
        }
        adsd3500_set_control("bitsInAB", value);
        std::cout << "bitsInAB was not found in .ini file" << std::endl;
    }

    // Configure Depth Enable and AB Averaging.
    it = iniKeyValPairs.find("partialDepthEnable");
    if (it != iniKeyValPairs.end()) {
        std::string en = (it->second == "0") ? "1" : "0";
        adsd3500_set_control("depthEnable", en);
        adsd3500_set_control("abAveraging", en);
    } else {
        std::cout << "partialDepthEnable was not found in .ini file" << std::endl;
    }

    return 0;
}


/*
*********************************Non-member functions*****************************
*/

// Print v4l2 buffer plane.
void print_planes(const struct v4l2_plane planes[], int num_planes) {
    for (int i = 0; i < num_planes; ++i) {
        std::cout << "Plane " << i << ":" << std::endl;
        std::cout << "  Bytesused: " << planes[i].bytesused << std::endl;
        std::cout << "  Length: " << planes[i].length << std::endl;
        std::cout << "  Data_offset: " << planes[i].data_offset << std::endl;
        std::cout << "  Reserved: " << planes[i].reserved[0] << " " << planes[i].reserved[1] << " "
                  << planes[i].reserved[2] << " " << planes[i].reserved[3] << std::endl;
    }
}

// Prints Byte Array
void PrintByteArray(unsigned char *byteArray, int arraySize) {
#ifdef DEBUG
    int i;
	for(i=0;i < arraySize; i++) 	{
		printf("0x%x ", byteArray[i]);
	}
	std::cout << std::endl;
#endif
}

// Sends an ioctl command to the ADSD3500 device
int xioctl(int fd, int request, void *arg) {
    int r;
    int tries = IOCTL_TRIES;
    do {
        r = ioctl(fd, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    return r;
}

// V4L2 Set command
bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val) {
	static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = CTRL_SIZE * sizeof(char);
    extCtrl.p_u8 = val;
    extCtrl.id = id;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;
    if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to set ctrl with id " << id << std::endl;
        return false;
    }

    return true;
}

// V4L2 Get command
bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val) {
	static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = CTRL_SIZE * sizeof(char);
    extCtrl.p_u8 = val;
    extCtrl.id = id;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;
    if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to get ctrl with id " << id;
        return false;
    }

    return true;
}

// Write Command
int32_t write_cmd(int fd, uint8_t *ptr, uint16_t len) {
    uint8_t cmd_data[CTRL_SIZE];
    if (ptr == nullptr) {
        return -1;
    }

    memcpy(&cmd_data[3], ptr, len);

    cmd_data[0] = 1;
    cmd_data[1] = (uint8_t)(len >> 8);
    cmd_data[2] = (uint8_t)(len & 0xFF);

    PrintByteArray(cmd_data, len + 3);
    int32_t ret = v4l2_ctrl_set(fd, 0x009819e1, cmd_data) ? 0 : -2;
    usleep(110 * 1000);
    return ret;
}

int32_t read_cmd(int fd, uint8_t *ptr, uint16_t len, uint8_t *rcmd,
                 uint16_t rlen) {
    uint8_t cmd_data[CTRL_SIZE];
    if (ptr == nullptr) {
        return -1;
    }

    memcpy(&cmd_data[3], ptr, len);

    cmd_data[0] = 1;
    cmd_data[1] = (uint8_t)(len >> 8);
    cmd_data[2] = (uint8_t)(len & 0xFF);

    PrintByteArray(cmd_data, len + 3);
    if (v4l2_ctrl_set(fd, 0x009819e1, cmd_data) == false) {
        return -1;
    }
    usleep(110 * 1000);

    cmd_data[0] = 0;
    cmd_data[1] = (uint8_t)(rlen >> 8);
    cmd_data[2] = (uint8_t)(rlen & 0xFF);
    PrintByteArray(cmd_data, len + 3);
    if (v4l2_ctrl_set(fd, 0x009819e1, cmd_data) == false) {
        return -2;
    }
    usleep(110 * 1000);
    int32_t ret = v4l2_ctrl_get(fd, 0x009819e1, cmd_data);

    uint16_t read_len = (cmd_data[1] << 8) | cmd_data[2];
    memcpy(rcmd, &cmd_data[3], std::min(rlen, read_len));

    return 0;
}

// Opens ToF Camera Device
int32_t tof_open(const char *tof_device) {
    int fd = ::open(tof_device, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        std::cout << "Failed to open the camera" << std::endl;
        return -1;
    }
    return fd;
}

