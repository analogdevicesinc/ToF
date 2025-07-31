/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>

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

#define IOCTL_TRIES 1
#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define BUF_SIZE 4096
#define CTRL_SIZE 4099

#ifdef NVIDIA
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
#endif

#ifdef NXP
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819e1)
#endif

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
		std::cout << "Failed to get ctrl with id " << id;
			return false;
	}

	return true;
}

int main(int argc, char **argv)
{

	uint8_t data[CTRL_SIZE] = {0};
	char binbuff[BUF_SIZE] = {0};
	uint32_t chunkSize = 2048u;
	uint32_t sizeOfBinary = 0u;
	uint32_t checksum = 0u;
	uint32_t packetsNeeded = 0u;

	int fd = open("/dev/v4l-subdev1", O_RDWR | O_NONBLOCK);
	if (fd == -1)
       	{
		std::cout << "Failed to open the camera" << std::endl;
		return -1;
	}
	
	std::ofstream fp(argv[1], std::ofstream::binary);
	if (!fp)
	{
		std::cout<<"Cannot open file!"<<std::endl;
		return 1;
	}
	
	//Find the binary file size
	fp.seekp(0, fp.beg);

	//Go to burst
	data[0] = 1; //WRITE
	data[2] = 4;
	data[3] = 0x00;
	data[4] = 0x19;
	data[5] = 0x00;
	data[6] = 0x00;
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);

	//Write header and readback response
	memset(data, 0x00, 16);
	data[0] = 1; //WRITE
	data[2] = 16;
	
	data[3] = 0xAD;
	data[6] = 0x13;
	data[11] = 0x13;
	data[15] = 1;
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);

	usleep(1000 * 1);	

	data[0] = 0;
	data[2] = 16;
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);

	v4l2_ctrl_get(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	
	std::cout<<"Done reading response header "<<std::endl;


	chunkSize = (data[5] << 8) | data[4];
	sizeOfBinary = (data[10] << 24) | (data[9] << 16) | (data[8] << 8) | data[7];

	for (uint32_t i=3; i<11; i++)
		checksum += data[i];

	packetsNeeded = sizeOfBinary / chunkSize;
	std::cout<<"Size of Binary :: "<<sizeOfBinary<<std::endl;
	std::cout<<"ChunkSize :: "<<chunkSize<<std::endl;
	std::cout<<"Packets Needed: "<<packetsNeeded + 1<<std::endl;

	for (uint8_t i=0; i<19; i++)
	{
		printf("0x%.2X\n", data[i]);
	}

	for (uint32_t i=1; i<=packetsNeeded; i++)
	{
		data[0] = 0;
		data[1] = chunkSize >> 8;
		data[2] = chunkSize & 0xFF;
		
		usleep(1000 * 30);

		bool retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
		v4l2_ctrl_get(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
		if (retval == false)
		{
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
		
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	v4l2_ctrl_get(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	memcpy(binbuff, &data[3], chunkSize);
	fp.write(binbuff, chunkSize - 4);
	std::cout<<"Packet number : "<<packetsNeeded + 1<<" / "<<packetsNeeded + 1<<std::endl;

	std::cout<<"Binary size : "<<sizeOfBinary<<std::endl;
	std::cout<<std::endl;

	//Exit burst
	memset(data, 0x00, 16);
	data[0] = 1; //WRITE
	data[2] = 16;
	data[3] = 0xAD;
	data[6] = 0x10;
	data[11] = 0x10;
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);

	fp.close();
	
	return 0;
}
