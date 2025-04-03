#ifndef NVM_TOOLS_COMMON_H
#define NVM_TOOLS_COMMON_H
#include "crc.h"
#include <algorithm>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <sstream>
#include <stdbool.h>
#include <stdint.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#pragma pack(push, 1)
typedef union {
    uint8_t cmd_header_byte[16];
    struct {
        uint8_t id8;                // 0xAD
        uint16_t chunk_size16;      // 256 is flash page size
        uint8_t cmd8;               // 0x04 is the CMD for fw upgrade
        uint32_t total_size_fw32;   // 4 bytes (total size of firmware)
        uint32_t header_checksum32; // 4 bytes header checksum
        uint32_t crc_of_fw32;       // 4 bytes CRC of the Firmware Binary
    };
} cmd_header_t;
#pragma pack(pop)

#define CTRL_SIZE 4099
#define IOCTL_TRIES 3
#define CLEAR(x) memset(&(x), 0, sizeof(x))

int32_t tof_open(const char *tof_device);
int xioctl(int fd, int request, void *arg);
bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val);
bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val);
void printByteArray(unsigned char *byteArray, int arraySize);
int32_t write_cmd(int fd, uint8_t *ptr, uint16_t len);
int32_t read_cmd(int fd, uint8_t *ptr, uint16_t len, uint8_t *rcmd,
                 uint16_t rlen);
int32_t send_firmware_update_command(int fd, uint8_t *fw_content,
                                     uint32_t fw_len);
int32_t get_firmware_version(int fd);
int32_t set_burst_mode(int fd);
void reset_adsd3500();
uint32_t getBinFile(std::ifstream &fp, uint8_t *buf);
void displayWaitMessage(const uint32_t wait);

#endif //NVM_TOOLS_COMMON_H