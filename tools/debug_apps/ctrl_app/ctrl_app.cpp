/****************************************************************************
# Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <malloc.h>
#include <cstring>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define IOCTL_TRIES 3
#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define CTRL_SIZE 4099
#define VER_MAJ 1
#define VER_MIN 0
#define VER_PATCH 0

static int xioctl(int fd, int request, void *arg)
{
	    int r;
	    int tries = IOCTL_TRIES;
	    do {
	        r = ioctl(fd, request, arg);
	    } while (--tries > 0 && r == -1 && EINTR == errno);

	    return r;
}

bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val)
{
		static struct v4l2_ext_control extCtrl;
		static struct v4l2_ext_controls extCtrls;

       	        extCtrl.size = CTRL_SIZE * sizeof(char);
		extCtrl.p_u8 = val;
		extCtrl.id = id;
		memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
		extCtrls.controls = &extCtrl;
		extCtrls.count = 1;
		if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
			std::cout << "Failed to set ctrl with id " << id<<std::endl;
				return false;
		}
		
		return true;
}

bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val)
{
	static struct v4l2_ext_control extCtrl;
	static struct v4l2_ext_controls extCtrls;

	extCtrl.size = CTRL_SIZE * sizeof(char);
	extCtrl.p_u8 = val;
	extCtrl.id = id;
	memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
	extCtrls.controls = &extCtrl;
	extCtrls.count = 1;
	if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
		std::cout << "Failed to get ctrl with id " << id<<std::endl;
			return false;
	}

	return true;
}


int main(int argc, char ** argv) {
	uint8_t data[CTRL_SIZE] = {0};

	int fd = open("/dev/v4l-subdev1", O_RDWR | O_NONBLOCK);
	if (fd == -1) {
		std::cout << "Failed to open the camera" << std::endl;
		return -1;
	}
	
	std::ifstream infile(argv[1]);
	std::string line;

	if (!infile.is_open()) {
		std::cout << "File infile.txt not found\n";
		return -1;
	}
	
	printf("Burst Control app version: %d.%d.%d\n", VER_MAJ, VER_MIN, VER_PATCH);

	while (std::getline(infile, line))
	{
		std::stringstream lineStream(line);
		std::string token;
		std::string r_w;
		int i = 0;
		lineStream >> r_w;
		while(lineStream >> token) {
			try {
				data[i+3] = stoi(token, 0, 16);
			}
			catch(std::invalid_argument& e){
			}
			i++;
			if (i > 4096) {
				std::cout << "Command line has to many bytes\n";
				return -1;
			}
		}

		data[0] = 1;
		data[1] = i >> 8;
		data[2] = i & 0xFF;
		
		v4l2_ctrl_set(fd, 0x009819e1, data);
		usleep(110 * 1000);
		if ((r_w == "R") && (i > 4)) {
			data[0] = 0;
			data[1] = data[4];
			data[2] = data[5];
			v4l2_ctrl_set(fd, 0x009819e1, data);
			usleep(110 * 1000);
			v4l2_ctrl_get(fd, 0x009819e1, data);

			int read_len = (data[1] << 8) | data[2];
			for (int j = 0; j < read_len; j++)
				printf("%02X ", data[j + 3]);
			printf("\n");
		} else if ((r_w == "R") && ((i == 2) || (i == 4))) {
			data[0] = 0;
			data[1] = 0;
			data[2] = 2;
			v4l2_ctrl_set(fd, 0x009819e1, data);
			usleep(110 * 1000);
			v4l2_ctrl_get(fd, 0x009819e1, data);

			int read_len = 2;

			for (int j = 0; j < read_len; j++)
				printf("%02X ", data[j + 3]);
			printf("\n");
		}
	}

	return 0;
}
