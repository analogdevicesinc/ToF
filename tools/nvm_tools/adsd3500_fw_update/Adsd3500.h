/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

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
#include <string>

bool validate_ext(std::string FileName);

#define ADI_DUAL_FW_SLOT_SIZE 0x20000  // 128 KB per slot
#define ADI_CHUNK_HEADER_SIZE 20       // ADI chunk header size in bytes

class Adsd3500 {
  public:
    Adsd3500(std::string FileName, bool force = false);

  private:
    int xioctl(int fd, int request, void *arg);

    std::string find_media_device_with_entity(const std::string &entity_name);

    std::string find_subdev_in_media(const std::string &media_dev,
                                     const std::string &entity_name);

    bool findDevicePathsAtVideo(const std::string &video,
                                std::string &subdev_path,
                                std::string &device_name);

    void open_device();

    bool Switch_from_Standard_to_Burst();

    bool Switch_from_Burst_to_Standard();

    bool Current_Firmware_Version(uint8_t cmd);
    bool Current_Firmware_Version(uint8_t cmd, uint8_t out_ver[44]);

    bool Read_Chip_ID(uint16_t reg_addr);

    bool updateAdsd3500MasterFirmware(uint8_t *fw_data, uint32_t fw_len, bool force, uint32_t expected_crc);

    bool updateAdsd3500SlaveFirmware(uint8_t *fw_data, uint32_t fw_len, bool force, uint32_t expected_crc);

    bool write_cmd(uint16_t cmd, uint16_t data);

    bool write_payload(uint8_t *payload, uint16_t payload_len);

    bool read_cmd(uint16_t cmd, uint16_t *data);

    bool read_burst_cmd(uint8_t *payload, uint16_t payload_len, uint8_t *data);

    std::string video = "/dev/media0";
    std::string deviceName = "adsd3500";
    std::string subdevPath;
    int sfd = -1;
    bool force = false;
};

#endif //FIRMWARE_UPDATE_ADSD3500_H
