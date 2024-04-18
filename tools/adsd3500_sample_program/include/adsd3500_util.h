/*****************************************************************************
* Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
* This software is proprietary & confidential to Analog Devices, Inc.
* and its licensors.
******************************************************************************
******************************************************************************
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


#ifndef FIRMWARE_UPDATE_ADSD3500_H
#define FIRMWARE_UPDATE_ADSD3500_H


#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <getopt.h> /* getopt_long() */

#include <fcntl.h> /* low-level i/o */
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <map>

#include <asm/types.h> /* for videodev2.h */

#include <linux/videodev2.h>

#include <cstdint>
#include <memory>
#include <string>
#include <iostream>
#include "../depthComputeLibrary/tofi_compute.h"
#include "../depthComputeLibrary/tofi_config.h"

#define ADSD3500_INTRINSIC_SIZE		56
#define ADSD3500_DEALIAS_SIZE		32
#define NUM_BUFFERS 4
#define ADSD3500_CTRL_PACKET_SIZE 4099
#define V4L2_CID_AD_DEV_CHIP_CONFIG (0x9819e1)
#define CTRL_SET_MODE (0x9819e0)
#define MAX_N_FREQS 3
#define MAX_N_MODES 10
#define MAX_CHAR_SIZE 12

#define MAX_SUBFRAMES_COUNT 10                                                 \
    // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define CAMERA_SENSOR_DRIVER "/dev/v4l-subdev1"
#define VIDEO_CAPTURE_DRIVER "/dev/video0"
#define CAPTURE_DEVICE_NAME "mxc-isi-cap"

struct buffer {
    void *start;
    size_t length;
};

struct VideoDev {
    int videoCaptureDeviceId;
    int cameraSensorDeviceId;
    struct buffer *videoBuffers;
    unsigned int nVideoBuffers;
    struct v4l2_plane planes[8];
    enum v4l2_buf_type videoBuffersType;
    bool started;

    VideoDev()
        : videoCaptureDeviceId(-1), cameraSensorDeviceId(-1), videoBuffers(nullptr), nVideoBuffers(0),
          started(false) {}
};

enum class ImagerType { IMAGER_UNKNOWN, IMAGER_ADSD3100, IMAGER_ADSD3030 };
enum class CCBVersion { CCB_UNKNOWN, CCB_VERSION0, CCB_VERSION1 };

class DealiasParams {
	public:
		int n_rows; 
 		int n_cols; 
 		uint8_t n_freqs; 
 		uint8_t row_bin_factor; 
 		uint8_t col_bin_factor; 
 		uint16_t n_offset_rows; 
 		uint16_t n_offset_cols; 
 		uint16_t n_sensor_rows; 
 		uint16_t n_sensor_cols; 
 		uint8_t FreqIndex[3]; 
 		uint16_t Freq[3]; 
};

class Adsd3500 {
	public:
		// Constructor
		Adsd3500();
		// Destructor
		~Adsd3500();

		int mode_num;
		TofiXYZDealiasData xyzDealiasData;
		ImagerType imagerType;
		CCBVersion ccbVersion;

		int OpenAdsd3500();
		int CloseAdsd3500();
		int ResetAdsd3500();
		int SetImageMode(uint8_t modeNumber);
		int GetImageMode(uint8_t* result);
		int StartStream();
		int StopStream();
		int SetFps(int fps);
		int GetFps(uint8_t* result);
		int ReadCCB(const char* filename);
		int ReadChipId(uint8_t* result);
		int ConfigureDeviceDrivers();
		int ConfigureAdsd3500WithIniParams();
		int ConfigureDepthComputeLibraryWithIniParams();
		int SetFrameType();
		int RequestFrame(uint16_t* buffer);
		int GetImagerTypeAndCCB();
		int GetIniKeyValuePairFromConfig(const char* iniFileName);
		int GetIntrinsicsAndDealiasParams();
		int ParseFramesWithDCL(uint16_t* buffer);

	private:
		ConfigFileData iniFileData;
		std::map<std::string, std::string> iniKeyValPairs;
		VideoDev videoDevice;
		int adsd3500_switch_to_burst_mode(int fd);
		int adsd3500_switch_to_standard_mode(int fd);
		int adsd3500_wait_for_buffer();
		int adsd3500_enqueue_internal_buffer(struct v4l2_buffer &buf);
		int adsd3500_dequeue_internal_buffer(struct v4l2_buffer &buf);
		int adsd3500_get_internal_buffer(uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf);
		int adsd3500_get_key_value_pairs_from_ini(const std::string &iniFileName, std::map<std::string, std::string> &iniKeyValPairs);
		int adsd3500_read_cmd(uint16_t cmd, uint16_t *data, unsigned int usDelay);
		int adsd3500_write_cmd(uint16_t cmd, uint16_t data);
		int adsd3500_read_payload_cmd(uint32_t cmd, uint8_t *readback_data, uint16_t payload_len);
		int adsd3500_read_payload(uint8_t *payload, uint16_t payload_len);
		int adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload, uint16_t payload_len);
		int adsd3500_write_payload(uint8_t *payload, uint16_t payload_len);
		int adsd3500_process_buffer(uint16_t *buffer = nullptr);

		int adsd3500_set_ini_params(const std::map<std::string, std::string> &iniKeyValPairs);
        int adsd3500_set_AB_invalidation_threshold(int threshold);
        int adsd3500_set_confidence_threshold(int threshold);
        int adsd3500_set_JBLF_filter_enableState(bool enable);
        int adsd3500_set_JBLF_filter_size(int size);
        int adsd3500_set_radial_threshold_min(int threshold);
        int adsd3500_set_radial_threshold_max(int threshold);
        int adsd3500_set_MIPI_output_speed(uint16_t speed);
        int adsd3500_set_VCSELDelay(uint16_t delay);
        int adsd3500_set_JBLF_max_edge_threshold(uint16_t threshold);
        int adsd3500_set_JBLF_AB_threshold(uint16_t threshold);
        int adsd3500_set_JBLF_gaussian_sigma(uint16_t value);
        int adsd3500_set_JBLF_exponential_term(uint16_t value);
        int adsd3500_set_frame_rate(uint16_t fps);
        int adsd3500_set_enable_edge_confidence(uint16_t value);
        int adsd3500_set_enable_phase_invalidation(uint16_t value);
};

// Non-member functions
void print_planes(const struct v4l2_plane planes[], int num_planes);
void PrintByteArray(unsigned char *byteArray, int arraySize);
int xioctl(int fd, int request, void *arg);
bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val);
bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val);
int32_t write_cmd(int fd, uint8_t *ptr, uint16_t len);
int32_t read_cmd(int fd, uint8_t *ptr, uint16_t len, uint8_t *rcmd, uint16_t rlen);
int32_t tof_open(const char *tof_device);

// Static functions
static uint32_t cal_crc32(uint32_t crc, unsigned char *buf, size_t len);

#endif
