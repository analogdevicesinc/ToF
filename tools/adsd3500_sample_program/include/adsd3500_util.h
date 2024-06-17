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

#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <malloc.h>
#include <map>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <asm/types.h> /* for videodev2.h */

#include <linux/videodev2.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <signal.h>
#include <string>

#include "../depthComputeLibrary/tofi_compute.h"
#include "../depthComputeLibrary/tofi_config.h"

#include "adsd3500_interrupt_notifier.h"

#define ADSD3500_INTRINSIC_SIZE 56
#define ADSD3500_DEALIAS_SIZE 32
#define ADSD3500_ADDR (0x70 >> 1)
#define ADSD3500_ADDR (0x70 >> 1)
#define ADSD3500_CTRL_PACKET_SIZE 4099
#define NUM_BUFFERS 4
#define MAX_N_FREQS 3
#define DEBUG 1
#define ADI_STATUS_FIRMWARE_UPDATE 0x000E
#define ADI_STATUS_NVM_WRITE_COMPLETE 0x000F
#define BUF_SIZE (4000000)
#define PAGE_SIZE 256u
#define IOCTL_TRIES 3
#define FULL_UPDATE_CMD 0x12
#define MAX_BIN_SIZE 741376
#define EMBEDDED_HEADER_SIZE 128

// V4L2 Camera Sensor Driver Control Commands
#define V4L2_CID_AD_DEV_CHIP_CONFIG (0x9819e1)
#define CTRL_SET_MODE (0x9819e0)         // Control to set Image Mode
#define CTRL_AB_AVG (0x9819e5)           // Control to enable AB Average
#define CTRL_DEPTH_EN (0x9819e6)         // Control to enable Depth
#define CTRL_PHASE_DEPTH_BITS (0x9819e2) // Control to set Phase Depth Bits
#define CTRL_AB_BITS (0x9819e3)          // Control to set AB bits
#define CTRL_CONFIDENCE_BITS (0x9819e4)  // Control to set Confidence Bits

#define MAX_SUBFRAMES_COUNT                                                    \
    10 // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

// Camera Sensor and Capture Driver Path.
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
        : videoCaptureDeviceId(-1), cameraSensorDeviceId(-1),
          videoBuffers(nullptr), nVideoBuffers(0), started(false) {}
};

/**
 * @struct Metadata
 * @brief Contains all of the metadata components
 */
struct Metadata {

    /**
    * @brief Width of frame
    */
    uint16_t width;

    /**
    * @brief Height of frame
    */
    uint16_t height;

    /**
    * @brief ADSD3500 Output Configuration:
    * 0 Full Depth Frame
    * 1 Phase Frame (Partial Depth)
    * 2 AB Frame
    * 3 Confidence Frame
    * 4 Depth AB Interleaved
    * 5 Phase and AB Interleaved
    * 6 Phase, JBLF Confidence and AB Interleaved
    * 7 Depth, Confidence and AB Interleaved
    */
    uint8_t outputConfiguration;

    /**
    * @brief Number of bits in depth
    */
    uint8_t bitsInDepth;

    /**
    * @brief Number of bits in AB
    */
    uint8_t bitsInAb;

    /**
    * @brief Number of bits in confidence
    */
    uint8_t bitsInConfidence;

    /**
    * @brief invalidPhaseValue:
    * In partial depth case, the host must know the invalid phase value used by the ADSD3500, which is used for invalidation during full depth compute.
    */
    uint16_t invalidPhaseValue;

    /**
    * @brief frequencyIndex: Stores index of the frequency for which the phase frame is outputted.
    */
    uint8_t frequencyIndex;

    /**
    * @brief abFrequencyIndex:
    * AB Frequency Index:
    * 0 AB of frequency 0
    * 1 AB of frequency 1
    * 2 AB of frequency 2
    * 3 AB Averaged
    */
    uint8_t abFrequencyIndex;

    /**
    * @brief Frame number
    */
    uint32_t frameNumber;

    /**
    * @brief Imager mode
    */
    uint8_t imagerMode;

    /**
    * @brief number of phases:
    * Number of phases in the input raw capture fed to the ADSD3500
    */
    uint8_t numberOfPhases;

    /**
    * @brief number of frequencies:
    * Number of frequencies in the input raw capture fed to the ADSD3500.
    */
    uint8_t numberOfFrequencies;

    /**
    * @brief True if xyz is being generated for the current frame. (set by sdk)
    */
    uint8_t xyzEnabled;

    /**
    * @brief elapsedTimeFractionalValue:
    * 32-bit fractional value out of total elapsed time.
    */
    uint32_t elapsedTimeFractionalValue;

    /**
    * @brief elapsedTimeSecondsValue:
    * 32-bit seconds value out of total elapsed time.
    */
    uint32_t elapsedTimeSecondsValue;

    /**
    * @brief Sensor temperature in degrees Celsius
    */
    int32_t sensorTemperature;

    /**
    * @brief Laser temperature in degrees Celsius
    */
    int32_t laserTemperature;
};

// Imager Types
enum class ImagerType { IMAGER_UNKNOWN, IMAGER_ADSD3100, IMAGER_ADSD3030 };
// CCB Versions
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

struct MODEMAP_TABLE_ENTRY {
    uint8_t UserDefinedMode;
    uint8_t CFGMode;
    uint16_t Height;
    uint16_t Width;
    uint8_t nFreq;
    uint8_t P0Mode;
    uint8_t TempMode;
    uint8_t INIIndex;
    uint16_t default_mode;
    uint16_t PassiveModeFlag;
    uint16_t nPhases;
    uint16_t spare3;
    uint16_t spare4;
    uint16_t spare5;
    uint16_t spare6;
    uint16_t spare7;
    uint16_t spare8;
};

struct INI_TABLE_ENTRY {
    uint8_t INIIndex;
    uint8_t rsvd; // for byte alignment of following fields
    uint16_t abThreshMin;
    uint16_t confThresh;
    uint16_t radialThreshMin;
    uint16_t radialThreshMax;
    uint16_t jblfApplyFlag;
    uint16_t jblfWindowSize;
    uint16_t jblfGaussianSigma;
    uint16_t jblfExponentialTerm;
    uint16_t jblfMaxEdge;
    uint16_t jblfABThreshold;
    uint16_t spare0;
    uint16_t spare1;
    uint16_t spare2;
    uint16_t spare3;
    uint16_t spare4;
    uint16_t spare5;
    uint16_t spare6;
    uint16_t spare7;
    uint16_t spare8;
};

struct Frame {
    uint16_t frameWidth;
    uint16_t frameHeight;
    __u32 pixelFormat;
};

class IniFilePath {
  public:
    // Mode to Config File Path Mapping for ADSD3030 (Tembin).
    static std::map<int, std::string> adsd3030ModeToConfigFileMap;

    // Mode to Config File Path Mapping for ADSD3100 (Crosby).
    static std::map<int, std::string> adsd3100ModeToConfigFileMap;
};

class Adsd3500InterruptNotifier;

class Adsd3500 : public std::enable_shared_from_this<Adsd3500> {
  public:
    // Constructor
    Adsd3500();
    // Destructor
    ~Adsd3500();

    int mode_num;
    TofiXYZDealiasData xyzDealiasData;
    ImagerType imagerType;
    CCBVersion ccbVersion;
    int depthBits = 0;
    int abBits = 0;
    int confBits = 0;
    std::string inputFormat;
    TofiComputeContext *tofi_compute_context;
    TofiConfig *tofi_config;
    int dynamic_mode_switch = 0;
    int ccb_as_master = 0;
    INI_TABLE_ENTRY ccb_iniTableEntry;
    Frame frame;
    int enableMetaDatainAB = 0;

    int OpenAdsd3500();
    int CloseAdsd3500();
    int ResetAdsd3500();
    int SetControl();
    int SetImageMode(uint8_t modeNumber);
    int GetImageMode(uint8_t *result);
    int StartStream();
    int StopStream();
    int SetFps(int fps);
    int GetFps(uint8_t *result);
    int ReadCCB(const char *filename);
    int ReadChipId(uint8_t *result);

    int ConfigureDeviceDrivers();
    int ConfigureAdsd3500WithIniParams();
    int ConfigureDepthComputeLibraryWithIniParams();
    int SetFrameType();
    int RequestFrame(uint16_t *buffer);
    int GetImagerTypeAndCCB();
    int GetIniKeyValuePair(const char *iniFileName);
    int GetIntrinsicsAndDealiasParams();
    int ParseRawDataWithDCL(uint16_t *buffer);
    int SetupInterruptSupport();
    int HandleInterrupts(int signalValue);
    int SubscribeSensorToNotifier();
    int StoreFrameMetaData(uint8_t *header_buffer, int num_frames);

  private:
    ConfigFileData iniFileData;
    std::map<std::string, std::string> iniKeyValPairs;
    VideoDev videoDevice;
    int adsd3500_switch_to_burst_mode(int fd);
    int adsd3500_switch_to_standard_mode(int fd);
    int adsd3500_wait_for_buffer();
    int adsd3500_enqueue_internal_buffer(struct v4l2_buffer &buf);
    int adsd3500_dequeue_internal_buffer(struct v4l2_buffer &buf);
    int adsd3500_get_internal_buffer(uint8_t **buffer, uint32_t &buf_data_len,
                                     const struct v4l2_buffer &buf);
    int adsd3500_get_ini_key_value_pairs_from_ini_file(
        const std::string &iniFileName);
    int adsd3500_get_ini_key_value_pairs_from_ccb();
    int adsd3500_update_ini_key_value_pairs(uint8_t *ini_table_buf,
                                            size_t buffer_size);
    int adsd3500_read_cmd(uint16_t cmd, uint16_t *data, unsigned int usDelay);
    int adsd3500_write_cmd(uint16_t cmd, uint16_t data);
    int adsd3500_read_payload_cmd(uint32_t cmd, uint8_t *readback_data,
                                  uint16_t payload_len);
    int adsd3500_read_payload(uint8_t *payload, uint16_t payload_len);
    int adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                   uint16_t payload_len);
    int adsd3500_write_payload(uint8_t *payload, uint16_t payload_len);
    int adsd3500_process_buffer(uint16_t *buffer = nullptr);
    int adsd3500_set_ini_params();
    int adsd3500_configure_sensor_frame_types();
    int adsd3500_set_control(const std::string &control,
                             const std::string &value);
    int adsd3500_configure_dynamic_mode_switching();
    int adsd3500_turnoff_dynamic_mode_switch();
    int adsd3500_set_enable_embedded_header_in_AB();

    std::shared_ptr<Adsd3500InterruptNotifier> adsd3500_interrupt_notifier;
};

// Non-member functions
void writeMetadataToFile(const Metadata &metadata, std::ofstream &outFile);
void print_planes(const struct v4l2_plane planes[], int num_planes);
void PrintByteArray(unsigned char *byteArray, int arraySize);
int xioctl(int fd, int request, void *arg);
bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val);
bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val);
int32_t write_cmd(int fd, uint8_t *ptr, uint16_t len);
int32_t read_cmd(int fd, uint8_t *ptr, uint16_t len, uint8_t *rcmd,
                 uint16_t rlen);
int32_t tof_open(const char *tof_device);
void PrintIniTableEntryFromCCB(struct INI_TABLE_ENTRY entry);

// Static functions
static uint32_t cal_crc32(uint32_t crc, unsigned char *buf, size_t len);

#endif
