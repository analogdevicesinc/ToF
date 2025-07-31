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

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <malloc.h>
#include <cstring>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include "include/compute_crc.h"
#define IOCTL_TRIES 1
#define USER_TASK _IOW('A',1,int32_t*)
#define SIGETX 		44
#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define PAGE_SIZE 256u
#define BUF_SIZE (4000000)
#define CTRL_SIZE 4099
#define ADSD3500_CTRL_PACKET_SIZE 4099
#define ADI_STATUS_NVM_FLASH_UPDATE	0x000F
#define FULL_UPDATE_CMD 0x12
#define VER_MAJ 1
#define VER_MIN 4
#define VER_PATCH 0
#define MAX_BIN_SIZE	1048576
#define MIN_BIN_SIZE    741376

#ifdef NVIDIA
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
#endif

#ifdef NXP
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819e1)
#endif

/* Seed value for CRC computation */
#define ADI_ROM_CFG_CRC_SEED_VALUE                      (0xFFFFFFFFu)

/* CRC32 Polynomial to be used for CRC computation */
#define ADI_ROM_CFG_CRC_POLYNOMIAL                      (0x04C11DB7u)

int fd;
int debug_fd;
static int handler_done = 0;
int signal_value = 0;
int Update_Complete = 0;

bool validate_ext(const char *filename){
    const char *dot = strrchr(filename, '.');
    if(strcmp(dot, ".nvm")){
		return false;
	}
	else{
		return true;
	}
	
}

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

void ctrl_c_handler(int n, siginfo_t *info, void *unused)
{
	if (n == SIGINT) {
		std::cout << "recieved ctrl-c"<< std::endl;
		handler_done = 1;
	}
}

void sig_event_handler(int n, siginfo_t *info, void *unused)
{
	if (n == SIGETX) {

		signal_value = info->si_int;
		/* std::cout << "Received signal from ADSD3500 kernel driver : Value =  " << signal_value << std::endl; */
		Update_Complete = 1;

	}
}

int main(int argc, char **argv)
{

	uint8_t data[CTRL_SIZE] = {0};
	uint8_t binbuff[BUF_SIZE] = {0};
	uint32_t sizeOfBinary = 0u;
	uint32_t actualSize = 0u;
	uint32_t packetsNeeded = 0u;
        uint32_t nResidualCRC = ADI_ROM_CFG_CRC_SEED_VALUE;
	int32_t number;
	uint8_t Wait_Time = 0;
	struct sigaction act;
	bool retval;
	bool validate;
	uint8_t Status_Command;

	if(argc < 2){
                std::cout<<"Please pass valid file.\nEg: Flash.nvm"<<std::endl;
                return 1;
        }

	validate = validate_ext(argv[1]);

	if( validate == false)
	{
		std::cout<<"Please pass valid file.\nEg: Flash.nvm"<<std::endl;
		return 1;
	}


	fd = open("/dev/v4l-subdev1", O_RDWR | O_NONBLOCK);
	if (fd == -1)
	{
		std::cout << "Failed to open the camera" << std::endl;
		return -1;
	}

	std::ifstream fp(argv[1], std::ifstream::binary);
	if (!fp)
	{
		std::cout<<"Cannot open file!"<<std::endl;
		return 1;
	}

	printf("NVM Flash update app version: %d.%d.%d\n", VER_MAJ, VER_MIN, VER_PATCH);

	/* install ctrl-c interrupt handler to cleanup at exit */
	sigemptyset (&act.sa_mask);
	act.sa_flags = (SA_SIGINFO | SA_RESETHAND);
	act.sa_sigaction = ctrl_c_handler;
	sigaction (SIGINT, &act, NULL);

	/* install custom signal handler */
	sigemptyset(&act.sa_mask);
	act.sa_flags = (SA_SIGINFO | SA_RESTART);
	act.sa_sigaction = sig_event_handler;
	sigaction(SIGETX, &act, NULL);

	std::cout << "Installed signal handler for SIGETX = "<< SIGETX << std::endl;

	/* Open ADSD3500 debugfs */
	debug_fd = open("/sys/kernel/debug/adsd3500/value", O_RDWR);
	if(debug_fd < 0) {
		std::cout << "Failed to open the debug sysfs " << std::endl;
		return -1;
	}

	if (ioctl(debug_fd, USER_TASK,(int32_t*) &number)) {
		printf("Failed to send IOCTL\n");
		std::cout << "Failed to send the IOCTL call" << std::endl;
		close(debug_fd);
		exit(1);
	}


	/* Find the binary file size */
	fp.seekg(0, fp.end);
	sizeOfBinary = fp.tellg();
	std::cout<<"Size of Binary: "<<sizeOfBinary<<std::endl;
	fp.seekg(0, fp.beg);


	if(sizeOfBinary < MIN_BIN_SIZE || sizeOfBinary > MAX_BIN_SIZE){
		std::cout<<"Failed to match the binary size "<< std::endl;
		std::cout<<"NVM Flash update failed " << std::endl;
		exit(0);
	}

	//Read the entire binary file into an intermediate buffer
	for (uint32_t i=0; i<sizeOfBinary; i++)
	{
		fp.read((char*)&binbuff[i],1);
	}

	if (sizeOfBinary % PAGE_SIZE)
		packetsNeeded = sizeOfBinary/PAGE_SIZE + 1;
	else
		packetsNeeded = sizeOfBinary/PAGE_SIZE;

	actualSize = packetsNeeded * PAGE_SIZE;

	std::cout<<"Packets Needed: "<<packetsNeeded<<std::endl;
	std::cout<<"Actual size (after padding): "<<actualSize<<std::endl;

	crc_parameters_t crc_params;
	crc_params.type = CRC_32bit;
	crc_params.polynomial.polynomial_crc32_bit = ADI_ROM_CFG_CRC_POLYNOMIAL;
	crc_params.initial_crc.crc_32bit = nResidualCRC;
	crc_params.crc_compute_flags = IS_CRC_MIRROR;

	crc_output_t res = compute_crc(&crc_params, binbuff, actualSize);
	nResidualCRC = ~res.crc_32bit;
	std::cout << std::hex << std::endl;
	std::cout << "Res " << nResidualCRC << std::endl;


	//Send SWITCH TO BURST command
	data[0] = 1;
	data[2] = 4;
	data[3] = 0x00;
	data[4] = 0x19;
	data[5] = 0x00;
	data[6] = 0x00;
	retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	usleep(110 * 1000);
	if (retval == false)
	{
		std::cout<< "ERROR! Could not send SWITCH_TO_BURST command!" <<std::endl;
		return -1;
	}
	memset(data, 0, 6);

	//Send UPDATE COMMAND header
	data[0] = 1;
	data[2] = 16;
	data[3] = 0xAD;
	data[4] = PAGE_SIZE & 0xFF;
	data[5] = PAGE_SIZE >> 8;
	data[6] = FULL_UPDATE_CMD;
	data[7] = actualSize & 0xFF;
	data[8] = actualSize >> 8;
	data[9] = actualSize >> 16;
	data[10] = actualSize >> 24;
	uint32_t checksum = 0;
	for (int i = 0; i < 7; i++){
		checksum += data[i+4];
	}
	memcpy(data + 11, &checksum, 4);
	memcpy(data + 15, &nResidualCRC, 4);

	std::cout<<"Header data: ";
	for (int i = 3; i < 19; i++)
		printf("%02x", data[i]);
	std::cout<<std::endl;

	retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	usleep(110 * 1000);
	if (retval == false)
	{
		std::cout<< "ERROR! Could not send header!" <<std::endl;
		return -1;
	}

	memset(data, 0, 19);
	std::cout << std::dec << std::endl;
	Update_Complete = 0;
	for (uint32_t i=0; i<packetsNeeded; i++)
	{
		data[0] = 1;
		data[1] = PAGE_SIZE >> 8;
		data[2] = PAGE_SIZE & 0xFF;

		memcpy(&data[3], &binbuff[i*PAGE_SIZE], PAGE_SIZE);

		retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
		if (retval == false)
		{
			return -1;
		}

		std::cout<<"Packet number: "<<i+1<<" / "<<packetsNeeded<< '\r';
		fflush(stdout);
	}
	std::cout<<std::endl;
	std::cout<<"Done"<<std::endl;
	std::cout<<"Waiting for the ADSD3500 kernel Driver signal "<<std::endl;
	while(1){
		if(Update_Complete != 0){
			std::cout << "Received signal from ADSD3500 kernel driver" << std::endl;
			break;
		}
		if(Wait_Time >= 30){
			std::cout<<"ADSD3500 kernel driver signal timeout occured"<<std::endl;
			break;
		}
		Wait_Time++;
		sleep(1);
	}

	sleep (2);

	//Send SWITCH TO STANDARD command
	memset(data, 0, 19);
	data[0] = 1;
	data[2] = 16;
	data[3] = 0xAD;
	data[6] = 0x10;
	data[11] = 0x10;
	data[15] = 0x01;
	retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	usleep(110 * 1000);
	if (retval == false)
	{
		std::cout<< "ERROR! Could not send SWITCH_TO_BURST command!" <<std::endl;
		return -1;
	}
	else
	{
		std::cout << std::endl;
		std::cout << "Switched from burst mode to standard mode"<< std::endl;
	}

	//Send GET STATUS command
	memset(data, 0, 19);
	data[0] = 1;
	data[1] = 0;
	data[2] = 2;
	data[3] = 0;
	data[4] = 32;
	retval = v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	usleep(110 * 1000);
	data[0] = 0;
	data[1] = 0;
	data[2] = 2;
	v4l2_ctrl_set(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);
	usleep(110 * 1000);
	v4l2_ctrl_get(fd, V4L2_CID_ADSD3500_DEV_CHIP_CONFIG, data);

	printf("Get Status: %02X\n", data[4]);
	Status_Command = data[4];

	if( Status_Command != ADI_STATUS_NVM_FLASH_UPDATE ){
		std::cout << "NVM Flash update failed" << std::endl;
	}
	else{
		std::cout << "NVM Flash update completed and success" << std::endl;
	}

	close(debug_fd);
	close(fd);
	return 0;
}
