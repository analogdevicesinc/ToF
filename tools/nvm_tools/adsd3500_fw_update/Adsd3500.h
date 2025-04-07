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

#include <fcntl.h> /* low-level i/o */
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include <asm/types.h> /* for videodev2.h */

#include <linux/videodev2.h>

#include <cstdint>
#include <memory>
#include <string>
#include <iostream>
class Adsd3500 {
	public:
		Adsd3500(std::string FileName);

	private:
		int xioctl(int fd, int request, void* arg);

		void open_device();

		void reverse(char *temp);

		int generate_mirror(int ch);

		bool Switch_from_Standard_to_Burst();

		bool Switch_from_Burst_to_Standard();

		bool Current_Firmware_Version();

		bool Read_Chip_ID();

		bool updateAdsd3500Firmware(const std::string &filePath);

		bool write_cmd(uint16_t cmd, uint16_t data);

		bool write_payload(uint8_t *payload, uint16_t payload_len);

		bool read_cmd(uint16_t cmd, uint16_t *data);

		bool read_burst_cmd(uint8_t *payload, uint16_t payload_len, uint8_t *data);

		const char *dev_name = "/dev/video0";
		const char *subdev_name = "/dev/v4l-subdev1";
		int fd = -1;
		int sfd = -1;
};


#endif //FIRMWARE_UPDATE_ADSD3500_H
