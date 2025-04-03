#include "nvm_tools_common.h"

uint8_t cmd_to_burst_mode[] = {0x00, 0x19, 0x00, 0x00};
uint8_t cmd_get_fw_version[] = {0xAD, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
                                0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

int32_t tof_open(const char *tof_device) {
    int fd = open(tof_device, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        std::cout << "Failed to open the camera" << std::endl;
        return -1;
    }
    return fd;
}

int xioctl(int fd, int request, void *arg) {
    int r;
    int tries = IOCTL_TRIES;
    do {
        r = ioctl(fd, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    return r;
}

bool v4l2_ctrl_set(int fd, uint32_t id, uint8_t *val) {
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;

    extCtrl.size = CTRL_SIZE * sizeof(char);
    extCtrl.p_u8 = val;
    extCtrl.id = id;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;
    if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Failed to set ctrl with id " << id << std::endl;
        return false;
    }

    return true;
}

bool v4l2_ctrl_get(int fd, uint32_t id, uint8_t *val) {
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

void printByteArray(unsigned char *byteArray, int arraySize) {
#ifdef DEBUG
    std::cout << "Byte Array (in Hexadecimal):" << std::endl;

    for (int i = 0; i < ((arraySize > 64) ? 64 : arraySize); i++) {
        std::cout << std::setw(2) << std::setfill('0') << std::hex
                  << static_cast<int>(byteArray[i]) << " ";
    }

    std::cout << std::dec << std::endl;
#endif
}

int32_t write_cmd(int fd, uint8_t *ptr, uint16_t len) {
    uint8_t cmd_data[CTRL_SIZE];
    if (ptr == nullptr) {
        return -1;
    }

    memcpy(&cmd_data[3], ptr, len);

    cmd_data[0] = 1;
    cmd_data[1] = (uint8_t)(len >> 8);
    cmd_data[2] = (uint8_t)(len & 0xFF);

    printByteArray(cmd_data, len + 3);
    int32_t ret = v4l2_ctrl_set(fd, 0x009819e1, cmd_data) ? 0 : -2;
    usleep(110 * 1000);
    return ret;
}

int32_t read_cmd(int fd, uint8_t *ptr, uint16_t len, uint8_t *rcmd,
                 uint16_t rlen) {
    uint8_t cmd_data[CTRL_SIZE];
    if (ptr == nullptr) {
        return -1;
    }

    memcpy(&cmd_data[3], ptr, len);

    cmd_data[0] = 1;
    cmd_data[1] = (uint8_t)(len >> 8);
    cmd_data[2] = (uint8_t)(len & 0xFF);

    printByteArray(cmd_data, len + 3);
    if (v4l2_ctrl_set(fd, 0x009819e1, cmd_data) == false) {
        return -1;
    }
    usleep(110 * 1000);

    cmd_data[0] = 0;
    cmd_data[1] = (uint8_t)(rlen >> 8);
    cmd_data[2] = (uint8_t)(rlen & 0xFF);
    printByteArray(cmd_data, len + 3);
    if (v4l2_ctrl_set(fd, 0x009819e1, cmd_data) == false) {
        return -2;
    }
    usleep(110 * 1000);
    int32_t ret = v4l2_ctrl_get(fd, 0x009819e1, cmd_data);

    uint16_t read_len = (cmd_data[1] << 8) | cmd_data[2];
    memcpy(rcmd, &cmd_data[3], std::min(rlen, read_len));

    return 0;
}

int32_t get_firmware_version(int fd) {
    const uint16_t rlen = 44;
    uint8_t read_buf[rlen];
    std::cout << "send_get_firmware_version_command:  ";
    int32_t ret =
        read_cmd(fd, cmd_get_fw_version,
                 sizeof(cmd_get_fw_version) / sizeof(cmd_get_fw_version[0]),
                 read_buf, rlen);
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    if (ret >= 0) {
        printByteArray(read_buf, rlen);
        std::cout << "Firmware Version: " << (uint16_t)read_buf[0] << "."
                  << (uint16_t)read_buf[1] << "." << (uint16_t)read_buf[2]
                  << std::endl;
    }
    return ret;
}

int32_t set_burst_mode(int fd) {
    std::cout << "send_switch_to_burst_mode_command;  ";
    int32_t ret =
        write_cmd(fd, cmd_to_burst_mode,
                  sizeof(cmd_to_burst_mode) / sizeof(cmd_to_burst_mode[0]));
    std::cout << ((ret >= 0) ? "SUCCESS" : "FAIL") << std::endl;
    return ret;
}

void reset_adsd3500() {
    std::cout << "Resetting the ADSD3500: ~7s." << std::endl;
    system("echo 0 > /sys/class/gpio/gpio122/value");
    usleep(100000);
    system("echo 1 > /sys/class/gpio/gpio122/value");
    usleep(7000000);
    std::cout << "Reset done." << std::endl;
}

uint32_t getBinFile(std::ifstream &fp, uint8_t *buf) {
    //Find the binary file size
    fp.seekg(0, fp.end);
    int32_t sizeOfBinary = fp.tellg();
    std::cout << "Size of Binary: " << sizeOfBinary << std::endl;
    fp.seekg(0, fp.beg);

    //Read the entire binary file into an intermediate buffer
    for (uint32_t i = 0; i < sizeOfBinary; i++) {
        fp.read((char *)&buf[i], 1);
    }

    return sizeOfBinary;
}

void displayWaitMessage(const uint32_t wait) {
    std::cout << "Waiting " << wait << "s " << std::flush;
    for (int cnt = 0; cnt < wait; cnt++) {
        std::cout << "." << std::flush;
        sleep(1);
    }
    std::cout << std::endl;
}