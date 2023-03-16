#include "nvm_tools_common.h"

//#define DEBUG

#define PAGE_SIZE 256u
#define BUF_SIZE (4000000)
#define FULL_UPDATE_CMD 0x12
#define POST_UPDATE_WAIT_SECONDS (20)

const char *CAMERA_DEV = "/dev/v4l-subdev1";

int32_t send_nvm_write_command(int fd, uint8_t *fw_content, uint32_t fw_len)
{
    uint8_t cmd_nvm_write[16] = {0};
	cmd_header_t *nvm_write_header = (cmd_header_t *)&cmd_nvm_write;

	nvm_write_header->id8 = 0xAD;
	nvm_write_header->chunk_size16 = PAGE_SIZE; // 256=0x100
	nvm_write_header->cmd8 = FULL_UPDATE_CMD;           // FW Upgrade CMD = 0x04
	nvm_write_header->total_size_fw32 = fw_len;
	nvm_write_header->header_checksum32 = 0;

	for (int i = 1; i < 8; i++) {
		nvm_write_header->header_checksum32 +=
			nvm_write_header->cmd_header_byte[i];
	}

	uint32_t res = crcFast(fw_content, fw_len, true) ^ 0xFFFFFFFF;
	nvm_write_header->crc_of_fw32 = ~res;

	std::cout << "send_nvm_write_commands: Update NVM command;  ";
	int32_t ret = write_cmd(fd, cmd_nvm_write, sizeof(cmd_nvm_write) / sizeof(cmd_nvm_write[0]));
	std::cout << ((ret >= 0)?"SUCCESS":"FAIL") << std::endl;
	return ret;
}

int main(int argc, char **argv) {

    uint8_t data[CTRL_SIZE] = {0};
    uint8_t binbuff[BUF_SIZE] = {0};
    uint32_t sizeOfBinary = 0u;
    uint32_t actualSize = 0u;
    uint32_t packetsNeeded = 0u;
    bool retval;

    std::ifstream fp(argv[1], std::ifstream::binary);
    if (!fp) {
        std::cout << "Usage: sudo " << argv[0] << " nvm_image.bin" << std::endl;
        return -2;
    }

    int32_t fd = tof_open(CAMERA_DEV);
    if (fd < 0)
    {
        std::cout << "Unable to find camera: " << CAMERA_DEV << std::endl;
        return -1;
    }

    sizeOfBinary = getBinFile(fp, binbuff);

    if (sizeOfBinary % PAGE_SIZE)
        packetsNeeded = sizeOfBinary / PAGE_SIZE + 1;
    else
        packetsNeeded = sizeOfBinary / PAGE_SIZE;

    actualSize = packetsNeeded * PAGE_SIZE;

    std::cout << "Packets Needed: " << packetsNeeded << std::endl;
    std::cout << "Actual size (after padding): " << actualSize << std::endl;

    reset_adsd3500();

    set_burst_mode(fd);

    if (send_nvm_write_command(fd, binbuff, actualSize) < 0)
    {
        return -3;
    }

    memset(data, 0, 19);

    for (uint32_t i = 0; i < packetsNeeded; i++) {
        data[0] = 1;
        data[1] = PAGE_SIZE >> 8;
        data[2] = PAGE_SIZE & 0xFF;

        memcpy(&data[3], &binbuff[i * PAGE_SIZE], PAGE_SIZE);

        retval = v4l2_ctrl_set(fd, 0x009819e1, data);
        if (retval == false) {
            return -4;
        }

        std::cout << "Packet number: " << i + 1 << " / " << packetsNeeded
                  << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;
    std::cout << "Done" << std::endl;

    displayWaitMessage(POST_UPDATE_WAIT_SECONDS);

    reset_adsd3500();

    set_burst_mode(fd);
	get_firmware_version(fd);

    reset_adsd3500();

    return 0;
}
