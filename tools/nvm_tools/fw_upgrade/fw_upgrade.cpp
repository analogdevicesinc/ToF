/*******************************************************************************
MIT License

Copyright (c) 2023 Analog Devices Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*******************************************************************************/

#include "nvm_tools_common.h"

//#define DEBUG

#define PAGE_SIZE 256u
#define BUF_SIZE (2000000)
#define POST_UPDATE_WAIT_SECONDS (60)

const char *CAMERA_DEV = "/dev/v4l-subdev1";

int32_t send_firmware_update_command(int fd, uint8_t *fw_content,
                                     uint32_t fw_len) {
    uint8_t cmd_fw_upgrade[16] = {0};
    cmd_header_t *fw_upgrade_header = (cmd_header_t *)&cmd_fw_upgrade;

    fw_upgrade_header->id8 = 0xAD;
    fw_upgrade_header->chunk_size16 = 0x0100; // 256=0x100
    fw_upgrade_header->cmd8 = 0x04;           // FW Upgrade CMD = 0x04
    fw_upgrade_header->total_size_fw32 = fw_len;
    fw_upgrade_header->header_checksum32 = 0;

    for (int i = 1; i < 8; i++) {
        fw_upgrade_header->header_checksum32 +=
            fw_upgrade_header->cmd_header_byte[i];
    }

    uint32_t res = crcFast(fw_content, fw_len, true) ^ 0xFFFFFFFF;
    fw_upgrade_header->crc_of_fw32 = ~res;

    std::cout << "send_init_update_commands: Update firmware command;  ";
    int32_t ret = write_cmd(fd, cmd_fw_upgrade,
                            sizeof(cmd_fw_upgrade) / sizeof(cmd_fw_upgrade[0]));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    return ret;
}

int main(int argc, char **argv) {

    uint8_t data[CTRL_SIZE] = {0};
    uint8_t binbuff[BUF_SIZE] = {0};
    uint32_t sizeOfBinary = 0u;
    uint32_t packetsNeeded = 0u;
    uint32_t bytesWritten = 0u;

    std::ifstream fp(argv[1], std::ifstream::binary);
    if (!fp) {
        std::cout << "Usage: sudo " << argv[0] << " firmware.bin" << std::endl;
        return -1;
    }

    int32_t fd = tof_open(CAMERA_DEV);
    if (fd < 0) {
        std::cout << "Unable to find camera: " << CAMERA_DEV << std::endl;
        return -1;
    }

    sizeOfBinary = getBinFile(fp, binbuff);

    //Read the entire binary file into an intermediate buffer
    for (uint32_t i = 0; i < sizeOfBinary; i++) {
        fp.read((char *)&binbuff[i], 1);
    }

    reset_adsd3500();

    set_burst_mode(fd);

    get_firmware_version(fd);

    if (send_firmware_update_command(fd, binbuff, sizeOfBinary) < 0) {
        return -3;
    }

    packetsNeeded = sizeOfBinary / PAGE_SIZE;
    std::cout << "Packets Needed: " << packetsNeeded << std::endl;

    for (uint32_t i = 0; i < packetsNeeded; i++) {
        data[0] = 1;
        data[1] = PAGE_SIZE >> 8;
        data[2] = PAGE_SIZE & 0xFF;

        memcpy(&data[3], &binbuff[i * PAGE_SIZE], PAGE_SIZE);

        bool retval = v4l2_ctrl_set(fd, 0x009819e1, data);
        if (!retval) {
            return -4;
        }

        std::cout << "Packet number: " << i + 1 << " / " << packetsNeeded
                  << '\r';
        fflush(stdout);
        sleep(0.10);
    }

    std::cout << std::endl << "BytesWritten: " << bytesWritten << std::endl;
    std::cout << "Binary size : " << sizeOfBinary << " Bytes" << std::endl;
    std::cout << std::endl;

    displayWaitMessage(POST_UPDATE_WAIT_SECONDS);

    set_burst_mode(fd);
    get_firmware_version(fd);

    reset_adsd3500();

    return 0;
}
