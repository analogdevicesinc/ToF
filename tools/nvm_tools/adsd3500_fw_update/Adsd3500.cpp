/****************************************************************************
# Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
# This software is proprietary & confidential to Analog Devices, Inc.
# and its licensors.
# *****************************************************************************
# *****************************************************************************/

#include "Adsd3500.h"
#include "compute_crc.h"
#include <array>
#include <fstream>

#include <iomanip>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <thread>
#include <vector>

#define FLASH_PAGE_SIZE 256
#define ADSD3500_CTRL_PACKET_SIZE 4099
#define WRITE_MASTER_FIRMWARE_COMMAND 0x04
#define WRITE_SLAVE_FIRMWARE_COMMAND 0x2A
#define GET_MASTER_FIRMWARE_COMMAND 0x01
#define GET_SLAVE_FIRMWARE_COMMAND 0x04
#define ADI_STATUS_FIRMWARE_UPDATE 0x000E
#define SET_SWITCH_TO_BURST_MODE 0x0019
#define GET_IMAGER_STATUS_CMD 0x0020
#define RESET_ADSD3500_CMD 0x0024
#define ADI_STATUS_SECOND_FIRMWARE_FLASH_UPDATE 0x0027
#define GET_MASTER_CHIP_ID_CMD 0x0112
#define GET_SLAVE_CHIP_ID_CMD 0x0116
#define ADSD3500_MASTER_CHIP_ID 0x5931
#define ADSD3500_SLAVE_CHIP_ID  0x5932
#define GET_DUAL_ADSD3500_ENABLED_CMD 0x005A  // Read: 0x0001=Dual Enabled, 0x0000=Dual Disabled
#define USER_TASK _IOW('A', 1, int32_t *)
#define SIGETX 44

#ifdef NVIDIA
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
const char *debugfs_name = "/proc/adsd3500/value";
#endif

#ifdef NXP
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819e1)
const char *debugfs_name = "/sys/kernel/debug/adsd3500/value";
#endif

#ifdef RPI
#define V4L2_CID_ADSD3500_DEV_CHIP_CONFIG (0x009819d1)
const char *debugfs_name = "/proc/adsd3500/value";
#endif

/* Seed value for CRC computation */
#define ADI_ROM_CFG_CRC_SEED_VALUE (0xFFFFFFFFu)

/* CRC32 Polynomial to be used for CRC computation */
#define ADI_ROM_CFG_CRC_POLYNOMIAL (0x04C11DB7u)

typedef union {
    uint8_t cmd_header_byte[16];
    struct __attribute__((__packed__)) {
        uint8_t id8;                // 0xAD
        uint16_t chunk_size16;      // 256 is flash page size
        uint8_t cmd8;               // 0x04 is the CMD for fw upgrade
        uint32_t total_size_fw32;   // 4 bytes (total size of firmware)
        uint32_t header_checksum32; // 4 bytes header checksum
        uint32_t crc_of_fw32;       // 4 bytes CRC of the Firmware Binary
    };
} cmd_header_t;

int debug_fd = -1;
static volatile sig_atomic_t handler_done = 0;
volatile sig_atomic_t signal_value = 0;
volatile sig_atomic_t Update_Complete = 0;

Adsd3500::Adsd3500(std::string FileName, bool force) {
    this->force = force;
    this->open_device();

    std::ifstream fw_file(FileName, std::ios::binary);
    if (!fw_file.is_open()) {
        std::cerr << "Error: cannot open firmware file '" << FileName << "'" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    std::vector<uint8_t> file_data(std::istreambuf_iterator<char>(fw_file), {});

    if (file_data.size() < 2 * ADI_DUAL_FW_SLOT_SIZE) {
        std::cerr << "Firmware file too small to contain both firmware slots" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Slot 0: master firmware (chunkId=0xAD, chunkType=0x54) at offset 0
    if (file_data[0] != 0xAD || file_data[1] != 0x54) {
        std::cerr << "Invalid Slot 0 header (expected chunkId=0xAD, chunkType=0x54)" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    uint32_t master_len =
        (uint32_t)file_data[8]               |
        ((uint32_t)file_data[9]  <<  8)      |
        ((uint32_t)file_data[10] << 16)      |
        ((uint32_t)file_data[11] << 24);
    if (master_len == 0 || master_len > ADI_DUAL_FW_SLOT_SIZE - ADI_CHUNK_HEADER_SIZE) {
        std::cerr << "Invalid master firmware size in Slot 0 header: " << master_len << " bytes" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Slot 1: slave firmware (chunkId=0xAD, chunkType=0x60) at offset ADI_DUAL_FW_SLOT_SIZE
    if (file_data[ADI_DUAL_FW_SLOT_SIZE] != 0xAD || file_data[ADI_DUAL_FW_SLOT_SIZE + 1] != 0x60) {
        std::cerr << "Invalid Slot 1 header (expected chunkId=0xAD, chunkType=0x60)" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    uint32_t slave_len =
        (uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + 8]              |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + 9]  <<  8)     |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + 10] << 16)     |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + 11] << 24);
    if (slave_len == 0 || slave_len > ADI_DUAL_FW_SLOT_SIZE - ADI_CHUNK_HEADER_SIZE) {
        std::cerr << "Invalid slave firmware size in Slot 1 header: " << slave_len << " bytes" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Extract master firmware payload from Slot 0 (after 20-byte chunk header)
    std::vector<uint8_t> master_fw(
        file_data.begin() + ADI_CHUNK_HEADER_SIZE,
        file_data.begin() + ADI_CHUNK_HEADER_SIZE + master_len);
    if (master_fw.size() != master_len) {
        std::cerr << "Master firmware buffer size mismatch: expected " << master_len
                  << " bytes, got " << master_fw.size() << " bytes" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Guard against null/zero-filled master firmware payload
    bool master_all_zero = true;
    for (uint32_t i = 0; i < master_len && master_all_zero; i++)
        if (master_fw[i] != 0x00) master_all_zero = false;
    if (master_all_zero) {
        std::cerr << "[ERR] Slot 0 master firmware payload is all zeros. "
                     "The .bin file may have been generated from a null stream. Aborting." << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Extract master CRC trailer (4 bytes immediately after the page-padded payload)
    uint32_t master_expected_crc =
        (uint32_t)file_data[ADI_CHUNK_HEADER_SIZE + master_len]            |
        ((uint32_t)file_data[ADI_CHUNK_HEADER_SIZE + master_len + 1] <<  8) |
        ((uint32_t)file_data[ADI_CHUNK_HEADER_SIZE + master_len + 2] << 16) |
        ((uint32_t)file_data[ADI_CHUNK_HEADER_SIZE + master_len + 3] << 24);
    std::cout << "[INFO] Header Master CRC : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << master_expected_crc << std::dec << std::endl;

    // Extract slave firmware payload from Slot 1 (after 20-byte chunk header)
    std::vector<uint8_t> slave_fw(
        file_data.begin() + ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE,
        file_data.begin() + ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE + slave_len);
    if (slave_fw.size() != slave_len) {
        std::cerr << "Slave firmware buffer size mismatch: expected " << slave_len
                  << " bytes, got " << slave_fw.size() << " bytes" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Guard against null/zero-filled slave firmware payload
    bool slave_all_zero = true;
    for (uint32_t i = 0; i < slave_len && slave_all_zero; i++)
        if (slave_fw[i] != 0x00) slave_all_zero = false;
    if (slave_all_zero) {
        std::cerr << "[ERR] Slot 1 slave firmware payload is all zeros. "
                     "The .bin file may have been generated from a null stream. Aborting." << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Extract slave CRC trailer (4 bytes immediately after the raw slave payload)
    uint32_t slave_expected_crc =
        (uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE + slave_len]            |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE + slave_len + 1] <<  8) |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE + slave_len + 2] << 16) |
        ((uint32_t)file_data[ADI_DUAL_FW_SLOT_SIZE + ADI_CHUNK_HEADER_SIZE + slave_len + 3] << 24);
    std::cout << "[INFO] Header Slave CRC  : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << slave_expected_crc << std::dec << std::endl;

    // Version check: first 4 bytes of both firmware payloads must match
    if (master_len < 4 || slave_len < 4) {
        std::cerr << "Firmware payload too small to contain a version number" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    char master_ver_str[32], slave_ver_str[32];
    snprintf(master_ver_str, sizeof(master_ver_str), "%d.%d.%d.%d",
             master_fw[0], master_fw[1], master_fw[2], master_fw[3]);
    snprintf(slave_ver_str,  sizeof(slave_ver_str),  "%d.%d.%d.%d",
             slave_fw[0],  slave_fw[1],  slave_fw[2],  slave_fw[3]);
    std::cout << "[INFO] Master firmware version : " << master_ver_str << std::endl;
    std::cout << "[INFO] Slave  firmware version : " << slave_ver_str  << std::endl;
    if (master_fw[0] != slave_fw[0] || master_fw[1] != slave_fw[1] ||
        master_fw[2] != slave_fw[2] || master_fw[3] != slave_fw[3]) {
        std::cerr << "[ERR] Version mismatch between master (" << master_ver_str
                  << ") and slave (" << slave_ver_str
                  << ") firmware payloads. Aborting." << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    std::cout << "[INFO] Firmware version match confirmed: " << master_ver_str << std::endl;

    // Probe master device — mandatory; validate chip ID matches ADSD3500_MASTER_CHIP_ID
    uint16_t master_chip_id = 0;
    bool master_found = read_cmd(GET_MASTER_CHIP_ID_CMD, &master_chip_id);
    if (master_found && master_chip_id == ADSD3500_MASTER_CHIP_ID) {
        std::cout << "Master Chip ID is: 0x" << std::hex << master_chip_id << std::dec << std::endl;
    } else {
        std::cerr << "No ADSD3500 master device detected (chip ID 0x"
                  << std::hex << master_chip_id
                  << " does not match expected 0x" << ADSD3500_MASTER_CHIP_ID
                  << "). Aborting firmware update." << std::dec << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    // Silent probe for slave — absence is expected in single-device configuration
    uint16_t slave_chip_id = 0;
    bool slave_found = read_cmd(GET_SLAVE_CHIP_ID_CMD, &slave_chip_id);
    if (slave_found && slave_chip_id == ADSD3500_SLAVE_CHIP_ID) {
        std::cout << std::endl << "Slave Chip ID is: 0x" << std::hex << slave_chip_id << std::dec << std::endl;
    } else {
        slave_found = false;  // reset: read failed or chip ID does not match ADSD3500_SLAVE_CHIP_ID
        // Slave chip ID read failed — slave may not be booted yet.
        // Query master to confirm whether dual ADSD3500 is enabled.
        uint16_t dual_enabled = 0;
        bool dual_query_ok = read_cmd(GET_DUAL_ADSD3500_ENABLED_CMD, &dual_enabled);
        if (dual_query_ok) {
            std::cout << "[INFO] Get Is Dual ADSD3500 Enabled (0x005A): 0x"
                      << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
                      << dual_enabled << std::dec << std::endl;
            if (dual_enabled == 0x0001) {
                std::cout << "[INFO] Dual ADSD3500 is enabled. Slave device confirmed via master query." << std::endl;
                slave_found = true;
            } else {
                std::cout << "[INFO] Dual ADSD3500 is disabled. Single-device configuration confirmed." << std::endl;
            }
        } else {
            std::cout << "[INFO] Slave chip ID read failed and dual-enable query also failed." << std::endl;
            std::cout << "[INFO] Assuming single-device configuration." << std::endl;
        }
    }

    if (master_found && slave_found) {
        std::cout << std::endl;
        std::cout << "Both ADSD3500 devices detected. Updating master and slave firmware." << std::endl;
        if (!this->updateAdsd3500MasterFirmware(master_fw.data(), master_len, force, master_expected_crc)) {
            std::cerr << "Master firmware update failed." << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
        if (!this->updateAdsd3500SlaveFirmware(slave_fw.data(), slave_len, force, slave_expected_crc)) {
            std::cerr << "Slave firmware update failed." << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
    } else {
        std::cout << std::endl;
        std::cout << "Single ADSD3500 device detected. Updating master firmware only." << std::endl;
        if (!this->updateAdsd3500MasterFirmware(master_fw.data(), master_len, force, master_expected_crc)) {
            std::cerr << "Master firmware update failed." << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
    }

    close(debug_fd);
    close(sfd);
}

bool validate_ext(std::string FileName) {
    const char *dot = strrchr(FileName.c_str(), '.');
    if (!dot || dot == FileName.c_str()) {
        return false;
    }
    return strcmp(dot, ".bin") == 0;
}

void ctrl_c_handler(int n, siginfo_t *info, void *unused) {
    if (n == SIGINT) {
        const char msg[] = "Received Ctrl-C\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        handler_done = 1;
    }
}

void sig_event_handler(int n, siginfo_t *info, void *unused) {
    if (n == SIGETX) {

        signal_value = info->si_int;
        /* std::cout << "Received signal from ADSD3500 kernel driver : Value =  " << signal_value << std::endl; */
        Update_Complete = 1;
    }
}

int Adsd3500::xioctl(int fd, int request, void *arg) {
    int r;

    do
        r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

std::string Adsd3500::find_media_device_with_entity(const std::string &entity_name)
{
    for (int i = 0; i <= 3; i++)
    {
        std::string media_dev = "/dev/media" + std::to_string(i);
        std::string cmd = "media-ctl -d " + media_dev + " --print-dot 2>/dev/null";

        std::array<char, 256> buffer{};
        std::string dot_output;

        FILE *pipe = popen(cmd.c_str(), "r");
        if (!pipe)
            continue;

        while (fgets(buffer.data(), buffer.size(), pipe))
            dot_output += buffer.data();

        pclose(pipe);

        if (dot_output.empty())
            continue;

        if (dot_output.find(entity_name) != std::string::npos)
            return media_dev;
    }

    return "";
}

std::string Adsd3500::find_subdev_in_media(const std::string &media_dev,
                                 const std::string &entity_name)
{
    std::string cmd = "media-ctl -d " + media_dev + " --print-dot 2>/dev/null";

    std::array<char, 256> buffer{};
    std::string dot;

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return "";

    while (fgets(buffer.data(), buffer.size(), pipe))
        dot += buffer.data();

    pclose(pipe);

    if (dot.empty())
        return "";

    size_t pos = dot.find(entity_name);
    if (pos == std::string::npos)
        return "";

    size_t dev_pos = dot.find("/dev/v4l-subdev", pos);
    if (dev_pos == std::string::npos)
        return "";

    size_t end = dot.find_first_of(" \"\n", dev_pos);
    return dot.substr(dev_pos, end - dev_pos);
}

bool Adsd3500::findDevicePathsAtVideo(const std::string &video,
                                      std::string &subdev_path,
                                      std::string &device_name) {

    char *buf;
    int size = 0;
    size_t pos;

    /* Run media-ctl to get the video processing pipes */
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "media-ctl -d %s --print-dot", video.c_str());
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        std::cout << "Error running media-ctl";
        return false;
    }

    /* Read the media-ctl output stream */
    buf = (char *)malloc(128 * 1024);
    if (!buf) {
        std::cerr << "Failed to allocate buffer for media-ctl output" << std::endl;
        pclose(fp);
        return false;
    }
    while (!feof(fp)) {
        size_t sz = fread(&buf[size], 1, 1, fp);
        if (sz > 0) {
            size++;
            if (size >= (128 * 1024 - 1)) {
                std::cerr << "media-ctl output exceeds buffer size" << std::endl;
                pclose(fp);
                free(buf);
                return false;
            }
        }
    }
    pclose(fp);
    buf[size] = '\0';

    /* Search command media-ctl for device/subdevice name */
    std::string str(buf);
    free(buf);

    if (str.find("adsd3500") != std::string::npos) {
        device_name = "adsd3500";
        pos = str.find("adsd3500");
        subdev_path = str.substr(pos + strlen("adsd3500") + 9);
        size_t end_pos = subdev_path.find_first_of(" \"\n");
        if (end_pos != std::string::npos)
            subdev_path = subdev_path.substr(0, end_pos);
    } else {
        return false;
    }
    return true;
}

void Adsd3500::open_device() {
    struct stat st;
    struct sigaction act;
    int32_t number = 0;
    bool status = true;

#if defined(NVIDIA) || defined(NXP)
    status = findDevicePathsAtVideo(video, subdevPath, deviceName);
    if (!status) {
        std::cerr << "Failed to find device paths at video: " << video << std::endl;
        exit(EXIT_FAILURE);
    }

#elif defined(RPI)
    const std::string target = "adsd3500";
    std::string media_dev = find_media_device_with_entity(target);

    if (media_dev.empty()) {
	    std::cout << "ADSD3500 not found in /dev/media0..media3" << std::endl;
	    exit(EXIT_FAILURE);
    }

    std::string subdevPath = find_subdev_in_media(media_dev, target);

    if (subdevPath.empty()) {
	    std::cout << "Could not find ADSD3500 v4l-subdev node" << std::endl;
	    exit(EXIT_FAILURE);
    }

    this->subdevPath = subdevPath;

#else
    #error "Unsupported platform: define NVIDIA, NXP, or RPI"
#endif

    /* Open V4L2 subdevice */
    if (stat(subdevPath.c_str(), &st) == -1) {
        std::cerr << "Failed to stat subdevice '" << subdevPath << "': " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        std::cerr << "'" << subdevPath << "' is not a character device" << std::endl;
        exit(EXIT_FAILURE);
    }

    sfd = open(subdevPath.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (sfd == -1) {
        std::cerr << "Failed to open subdevice '" << subdevPath << "': " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    /* install ctrl-c interrupt handler to cleanup at exit */
    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESETHAND);
    act.sa_sigaction = ctrl_c_handler;
    sigaction(SIGINT, &act, NULL);

    /* install custom signal handler */
    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESTART);
    act.sa_sigaction = sig_event_handler;
    sigaction(SIGETX, &act, NULL);

    std::cout << "Installed signal handler for SIGETX = " << SIGETX
              << std::endl;

    /* Open ADSD3500 debugfs */
    debug_fd = open(debugfs_name, O_RDWR);
    if (debug_fd < 0) {
        std::cerr << "Failed to open the debug sysfs" << std::endl;
        close(sfd);
        exit(EXIT_FAILURE);
    }

    if (ioctl(debug_fd, USER_TASK, (int32_t *)&number)) {
        printf("Failed to send IOCTL\n");
        std::cout << "Failed to send the IOCTL call" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
}

bool Adsd3500::updateAdsd3500MasterFirmware(uint8_t *fw_data, uint32_t fw_len, bool force, uint32_t expected_crc) {
    bool status = true;
    uint8_t Wait_Time = 0;
    uint16_t Status_Command = 0;
    uint32_t nResidualCRC = ADI_ROM_CFG_CRC_SEED_VALUE;

    std::cout << std::endl;
    std::cout << "===== updateAdsd3500MasterFirmware: Starting Master Firmware Update =====" << std::endl;
    std::cout << "[MASTER] ";
    Read_Chip_ID(GET_MASTER_CHIP_ID_CMD);
    sleep(1);

    std::cout << std::dec;

    if (!Switch_from_Standard_to_Burst()) {
        std::cerr << "Master: failed to switch to burst mode before firmware write" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl;
    std::cout << "[MASTER] Before upgrading new firmware ";
    uint8_t current_ver[44] = {0};
    Current_Firmware_Version(GET_MASTER_FIRMWARE_COMMAND, current_ver);
    sleep(1);

    // Version downgrade check
    if (fw_len >= 4) {
        char update_ver_str[32], current_ver_str[32];
        snprintf(update_ver_str, sizeof(update_ver_str), "%d.%d.%d.%d",
                 fw_data[0], fw_data[1], fw_data[2], fw_data[3]);
        snprintf(current_ver_str, sizeof(current_ver_str), "%d.%d.%d.%d",
                 current_ver[0], current_ver[1], current_ver[2], current_ver[3]);
        std::cout << "[MASTER] Update firmware version  : " << update_ver_str << std::endl;

        // Minimum version check: current device firmware must be >= 8.1.0.0
        const uint8_t min_ver[4] = {8, 1, 0, 0};
        bool below_minimum = false;
        for (int i = 0; i < 4; i++) {
            if (current_ver[i] < min_ver[i]) { below_minimum = true;  break; }
            if (current_ver[i] > min_ver[i]) { below_minimum = false; break; }
        }
        if (below_minimum) {
            std::cerr << "[MASTER] ERROR: Current device firmware version " << current_ver_str
                      << " is below the minimum required version 8.1.0.0. Aborting." << std::endl;
            Switch_from_Burst_to_Standard();
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }

        bool is_downgrade = false;
        for (int i = 0; i < 4; i++) {
            if (fw_data[i] < current_ver[i]) { is_downgrade = true;  break; }
            if (fw_data[i] > current_ver[i]) { is_downgrade = false; break; }
        }

        if (is_downgrade) {
            std::cerr << std::endl;
            std::cerr << "[MASTER] WARNING: Downgrade detected for master firmware!" << std::endl;
            std::cerr << "  Current version : " << current_ver_str << std::endl;
            std::cerr << "  Update version  : " << update_ver_str << std::endl;
            if (!force) {
                std::cerr << "Downgrade requires explicit confirmation." << std::endl;
                std::cerr << "Re-run with --force to proceed." << std::endl;
                Switch_from_Burst_to_Standard();
                close(debug_fd);
                close(sfd);
                exit(2);
            }
            std::cerr << "[MASTER] Proceeding with downgrade (--force specified)." << std::endl;
        }
    }

    cmd_header_t fw_upgrade_header;
    fw_upgrade_header.id8 = 0xAD;
    fw_upgrade_header.chunk_size16 = 0x0100; // 256=0x100
    fw_upgrade_header.cmd8 = WRITE_MASTER_FIRMWARE_COMMAND;
    fw_upgrade_header.total_size_fw32 = fw_len;
    fw_upgrade_header.header_checksum32 = 0;

    for (int i = 1; i < 8; i++) {
        fw_upgrade_header.header_checksum32 +=
            fw_upgrade_header.cmd_header_byte[i];
    }

    crc_parameters_t crc_params;
    crc_params.type = CRC_32bit;
    crc_params.polynomial.polynomial_crc32_bit = ADI_ROM_CFG_CRC_POLYNOMIAL;
    crc_params.initial_crc.crc_32bit = nResidualCRC;
    crc_params.crc_compute_flags = IS_CRC_MIRROR;

    crc_output_t res = compute_crc(&crc_params, fw_data, fw_len);
    nResidualCRC = ~res.crc_32bit;
    std::cout << "[MASTER] CRC raw result : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << res.crc_32bit << std::dec << std::endl;
    std::cout << "[MASTER] nResidualCRC   : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << nResidualCRC << std::dec << std::endl;
    std::cout << "[MASTER] Expected CRC   : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << expected_crc << std::dec << std::endl;
    if (nResidualCRC != expected_crc) {
        std::cerr << "[MASTER] CRC MISMATCH: computed 0x"
                  << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
                  << nResidualCRC << " != expected 0x"
                  << std::setw(8) << std::setfill('0')
                  << expected_crc << std::dec << std::endl;
        return false;
    }
    std::cout << "[MASTER] CRC OK: computed CRC matches expected CRC." << std::endl;

    fw_upgrade_header.crc_of_fw32 = (uint32_t)nResidualCRC;

    status = write_payload(fw_upgrade_header.cmd_header_byte, 16);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "[MASTER] Failed to send fw upgrade header" << std::endl;
        return status;
    }

    int packetsToSend;
    if ((fw_len % FLASH_PAGE_SIZE) != 0) {
        packetsToSend = (fw_len / FLASH_PAGE_SIZE + 1);
    } else {
        packetsToSend = (fw_len / FLASH_PAGE_SIZE);
    }

    uint8_t data_out[FLASH_PAGE_SIZE];

    std::cout << std::endl;
    std::cout << "[MASTER] Writing Firmware packets..." << std::endl;
    Update_Complete = 0;
    for (int i = 0; i < packetsToSend; i++) {
        int start = FLASH_PAGE_SIZE * i;
        int end = FLASH_PAGE_SIZE * (i + 1);

        for (int j = start; j < end; j++) {
            if (j < (int)fw_len) {
                data_out[j - start] = fw_data[j];
            } else {
                data_out[j - start] = 0x00;
            }
        }
        status = write_payload(data_out, FLASH_PAGE_SIZE);

        if (!status) {
            std::cerr << "[MASTER] Failed to send packet number " << i << " out of "
                      << packetsToSend << " packets!" << std::endl;
            return status;
        }

        std::cout << "[MASTER] Packet number: " << i + 1 << " / " << packetsToSend
                  << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "[MASTER] Adsd3500 master firmware packets sent successfully!" << std::endl;

    std::cout << std::endl;
    std::cout << "[MASTER] Waiting for the ADSD3500 kernel Driver signal " << std::endl;

    while (1) {
        if (handler_done) {
            std::cerr << "\nAborted by user (Ctrl-C)" << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
        if (Update_Complete != 0) {
            std::cout << "[MASTER] Received signal from ADSD3500 kernel driver"
                      << std::endl;
            break;
        }
        if (Wait_Time >= 30) {
            std::cout << "[MASTER] ADSD3500 kernel driver signal timeout occurred"
                      << std::endl;
            status = read_cmd(GET_IMAGER_STATUS_CMD, &Status_Command);
            std::cout << std::hex;
            std::cout << "[MASTER] Get status Command " << Status_Command << std::endl;

            std::cout << "[MASTER] Firmware update failed" << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
        Wait_Time++;
        sleep(1);
    }

    std::cout << std::endl;
    for (int i = 9; i >= 0; i--) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[MASTER] Waiting for " << i << " seconds" << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;

    status = read_cmd(GET_IMAGER_STATUS_CMD, &Status_Command);
    std::cout << "[MASTER] Get status Command 0x"
              << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << Status_Command << std::dec << std::endl;

    if (Status_Command != ADI_STATUS_FIRMWARE_UPDATE) {
        std::cout << "[MASTER] Firmware update failed" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }

    sleep(2);

    /*Soft Reset the ADSD3500*/
    status = write_cmd(RESET_ADSD3500_CMD, 0x0000);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to Soft Reset the ADSD3500!" << std::endl;
        return status;
    } else {
        std::cout << std::endl;
        std::cout << "[MASTER] Firmware soft resetting...";
    }

    std::cout << std::endl;
    for (int i = 9; i >= 0; i--) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[MASTER] Waiting for " << i << " seconds" << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;

    std::cout << "[MASTER] ";
    Read_Chip_ID(GET_MASTER_CHIP_ID_CMD);
    sleep(1);

    if (!Switch_from_Standard_to_Burst()) {
        std::cerr << "Master: failed to switch to burst mode during post-flash verification" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl;
    std::cout << "[MASTER] After upgrading new firmware ";
    Current_Firmware_Version(GET_MASTER_FIRMWARE_COMMAND);
    sleep(1);

    if (!Switch_from_Burst_to_Standard()) {
        std::cerr << "Master: failed to switch back to standard mode after post-flash verification" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl << "[MASTER] ";
    Read_Chip_ID(GET_MASTER_CHIP_ID_CMD);

    return true;
}

bool Adsd3500::updateAdsd3500SlaveFirmware(uint8_t *fw_data, uint32_t fw_len, bool force, uint32_t expected_crc) {

    bool status = true;
    uint8_t Wait_Time = 0;
    uint16_t Status_Command = 0;
    uint32_t nResidualCRC = ADI_ROM_CFG_CRC_SEED_VALUE;

    std::cout << std::endl;
    std::cout << "===== updateAdsd3500SlaveFirmware: Starting Slave Firmware Update =====" << std::endl;
    std::cout << "[SLAVE] ";
    Read_Chip_ID(GET_SLAVE_CHIP_ID_CMD);
    sleep(1);

    std::cout << std::dec;

    if (!Switch_from_Standard_to_Burst()) {
        std::cerr << "Slave: failed to switch to burst mode before firmware write" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl;
    std::cout << "[SLAVE] Before upgrading new firmware ";
    uint8_t current_ver[44] = {0};
    Current_Firmware_Version(GET_SLAVE_FIRMWARE_COMMAND, current_ver);
    sleep(1);

    // Version downgrade check
    if (fw_len >= 4) {
        char update_ver_str[32], current_ver_str[32];
        snprintf(update_ver_str, sizeof(update_ver_str), "%d.%d.%d.%d",
                 fw_data[0], fw_data[1], fw_data[2], fw_data[3]);
        snprintf(current_ver_str, sizeof(current_ver_str), "%d.%d.%d.%d",
                 current_ver[0], current_ver[1], current_ver[2], current_ver[3]);
        std::cout << "[SLAVE] Update firmware version  : " << update_ver_str << std::endl;

        // Minimum version check: current device firmware must be >= 8.1.0.0
        const uint8_t min_ver[4] = {8, 1, 0, 0};
        bool below_minimum = false;
        for (int i = 0; i < 4; i++) {
            if (current_ver[i] < min_ver[i]) { below_minimum = true;  break; }
            if (current_ver[i] > min_ver[i]) { below_minimum = false; break; }
        }
        if (below_minimum) {
            std::cerr << "[SLAVE] ERROR: Current device firmware version " << current_ver_str
                      << " is below the minimum required version 8.1.0.0. Aborting." << std::endl;
            Switch_from_Burst_to_Standard();
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }

        bool is_downgrade = false;
        for (int i = 0; i < 4; i++) {
            if (fw_data[i] < current_ver[i]) { is_downgrade = true;  break; }
            if (fw_data[i] > current_ver[i]) { is_downgrade = false; break; }
        }

        if (is_downgrade) {
            std::cerr << std::endl;
            std::cerr << "[SLAVE] WARNING: Downgrade detected for slave firmware!" << std::endl;
            std::cerr << "  Current version : " << current_ver_str << std::endl;
            std::cerr << "  Update version  : " << update_ver_str << std::endl;
            if (!force) {
                std::cerr << "Downgrade requires explicit confirmation." << std::endl;
                std::cerr << "Re-run with --force to proceed." << std::endl;
                Switch_from_Burst_to_Standard();
                close(debug_fd);
                close(sfd);
                exit(2);
            }
            std::cerr << "[SLAVE] Proceeding with downgrade (--force specified)." << std::endl;
        }
    }

    cmd_header_t fw_upgrade_header;
    fw_upgrade_header.id8 = 0xAD;
    fw_upgrade_header.chunk_size16 = 0x0100; // 256=0x100
    fw_upgrade_header.cmd8 = WRITE_SLAVE_FIRMWARE_COMMAND;
    fw_upgrade_header.total_size_fw32 = fw_len;
    fw_upgrade_header.header_checksum32 = 0;

    for (int i = 1; i < 8; i++) {
        fw_upgrade_header.header_checksum32 +=
            fw_upgrade_header.cmd_header_byte[i];
    }

    crc_parameters_t crc_params;
    crc_params.type = CRC_32bit;
    crc_params.polynomial.polynomial_crc32_bit = ADI_ROM_CFG_CRC_POLYNOMIAL;
    crc_params.initial_crc.crc_32bit = nResidualCRC;
    crc_params.crc_compute_flags = IS_CRC_MIRROR;

    crc_output_t res = compute_crc(&crc_params, fw_data, fw_len);
    nResidualCRC = ~res.crc_32bit;
    std::cout << "[SLAVE] CRC raw result : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << res.crc_32bit << std::dec << std::endl;
    std::cout << "[SLAVE] nResidualCRC   : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << nResidualCRC << std::dec << std::endl;
    std::cout << "[SLAVE] Expected CRC   : 0x"
              << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
              << expected_crc << std::dec << std::endl;
    if (nResidualCRC != expected_crc) {
        std::cerr << "[SLAVE] CRC MISMATCH: computed 0x"
                  << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
                  << nResidualCRC << " != expected 0x"
                  << std::setw(8) << std::setfill('0')
                  << expected_crc << std::dec << std::endl;
        return false;
    }
    std::cout << "[SLAVE] CRC OK: computed CRC matches expected CRC." << std::endl;

    fw_upgrade_header.crc_of_fw32 = (uint32_t)nResidualCRC;

    status = write_payload(fw_upgrade_header.cmd_header_byte, 16);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "[SLAVE] Failed to send fw upgrade header" << std::endl;
        return status;
    }

    int packetsToSend;
    if ((fw_len % FLASH_PAGE_SIZE) != 0) {
        packetsToSend = (fw_len / FLASH_PAGE_SIZE + 1);
    } else {
        packetsToSend = (fw_len / FLASH_PAGE_SIZE);
    }

    uint8_t data_out[FLASH_PAGE_SIZE];

    std::cout << std::endl;
    std::cout << "[SLAVE] Writing Firmware packets..." << std::endl;
    Update_Complete = 0;
    for (int i = 0; i < packetsToSend; i++) {
        int start = FLASH_PAGE_SIZE * i;
        int end = FLASH_PAGE_SIZE * (i + 1);

        for (int j = start; j < end; j++) {
            if (j < (int)fw_len) {
                data_out[j - start] = fw_data[j];
            } else {
                data_out[j - start] = 0x00;
            }
        }
        status = write_payload(data_out, FLASH_PAGE_SIZE);

        if (!status) {
            std::cerr << "[SLAVE] Failed to send packet number " << i << " out of "
                      << packetsToSend << " packets!" << std::endl;
            return status;
        }

        std::cout << "[SLAVE] Packet number: " << i + 1 << " / " << packetsToSend
                  << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "[SLAVE] Adsd3500 slave firmware packets sent successfully!" << std::endl;

    std::cout << std::endl;
    std::cout << "[SLAVE] Waiting for the ADSD3500 kernel Driver signal " << std::endl;

    while (1) {
        if (handler_done) {
            std::cerr << "\nAborted by user (Ctrl-C)" << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
        if (Update_Complete != 0) {
            std::cout << "[SLAVE] Received signal from ADSD3500 kernel driver"
                      << std::endl;
            break;
        }
        if (Wait_Time >= 30) {
            std::cout << "[SLAVE] ADSD3500 kernel driver signal timeout occurred"
                      << std::endl;
            status = read_cmd(GET_IMAGER_STATUS_CMD, &Status_Command);
            std::cout << "[SLAVE] Get status Command 0x"
                      << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << Status_Command << std::dec << std::endl;

            std::cout << "[SLAVE] Firmware update failed" << std::endl;
            close(debug_fd);
            close(sfd);
            exit(EXIT_FAILURE);
        }
        Wait_Time++;
        sleep(1);
    }

    sleep(2);

    if (!Switch_from_Burst_to_Standard()) {
        std::cerr << "Slave: failed to switch back to standard mode before reading flash status" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    sleep(1);

    status = read_cmd(GET_IMAGER_STATUS_CMD, &Status_Command);
    std::cout << "[SLAVE] Get status Command 0x"
              << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << Status_Command << std::dec << std::endl;

    if (Status_Command != ADI_STATUS_SECOND_FIRMWARE_FLASH_UPDATE) {
        std::cout << "Slave Firmware write failed" << std::endl;
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    } else {
        std::cout << "Slave Firmware Flash write completed and is successful."
                  << std::endl;
    }

    /*Soft Reset the ADSD3500*/
    status = write_cmd(RESET_ADSD3500_CMD, 0x0000);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to Soft Reset the ADSD3500!" << std::endl;
        return status;
    } else {
        std::cout << std::endl;
        std::cout << "[SLAVE] Firmware soft resetting..." << std::endl;
    }

    std::cout << std::endl;
    for (int i = 9; i >= 0; i--) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[SLAVE] Waiting for " << i << " seconds" << '\r';
        fflush(stdout);
    }
    std::cout << std::endl;

    std::cout << "[SLAVE] ";
    Read_Chip_ID(GET_SLAVE_CHIP_ID_CMD);
    sleep(1);

    if (!Switch_from_Standard_to_Burst()) {
        std::cerr << "Slave: failed to switch to burst mode during post-flash verification" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl;
    std::cout << "[SLAVE] After upgrading new firmware ";
    Current_Firmware_Version(GET_SLAVE_FIRMWARE_COMMAND);
    sleep(1);

    if (!Switch_from_Burst_to_Standard()) {
        std::cerr << "Slave: failed to switch back to standard mode after post-flash verification" << std::endl;
        return false;
    }
    sleep(1);

    std::cout << std::endl << "[SLAVE] ";
    Read_Chip_ID(GET_SLAVE_CHIP_ID_CMD);

    return true;
}

bool Adsd3500::Read_Chip_ID(uint16_t reg_addr) {
    bool status = true;
    // Read Chip ID in STANDARD mode
    uint16_t chip_id;
    status = read_cmd(reg_addr, &chip_id);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to read adsd3500 chip id!" << std::endl;
        return status;
    }

    std::cout << "Chip ID is: " << std::hex << chip_id << std::endl;
    return status;
}

bool Adsd3500::Switch_from_Standard_to_Burst() {
    bool status = true;

    // Switch to BURST mode.
    status = write_cmd(SET_SWITCH_TO_BURST_MODE, 0x0000);
    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to switch to burst mode!" << std::endl;
        return status;
    } else {
        std::cout << std::endl;
        std::cout << "Switched from standard mode to burst mode" << std::endl;
    }
    return status;
}
bool Adsd3500::Switch_from_Burst_to_Standard() {

    bool status = true;
    /*Commands to switch back to standard mode*/
    uint8_t switchBuf[] = {0xAD, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
                           0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    status = write_payload(switchBuf, sizeof(switchBuf) / sizeof(switchBuf[0]));
    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to switch adsd3500 to standard mode!" << std::endl;
        return status;
    } else {
        std::cout << std::endl;
        std::cout << "Switched from burst mode to standard mode" << std::endl;
    }
    return status;
}
bool Adsd3500::Current_Firmware_Version(uint8_t cmd) {
    return Current_Firmware_Version(cmd, nullptr);
}

bool Adsd3500::Current_Firmware_Version(uint8_t cmd, uint8_t out_ver[44]) {

    bool status = true;
    uint8_t Current_FW_Version[44] = {0};
    char version[16];

    // Read Current Firmware version
    uint8_t current_fw_version_command[] = {0xAD, 0x00, 0x2C, 0x05, 0x00, 0x00,
                                            0x00, 0x00, 0x31, 0x00, 0x00, 0x00,
                                            0x01, 0x00, 0x00, 0x00};
    current_fw_version_command[12] = cmd;
    status = read_burst_cmd(current_fw_version_command,
                            sizeof(current_fw_version_command) /
                                sizeof(current_fw_version_command[0]),
                            Current_FW_Version);

    if (!status) {
        std::cout << std::endl;
        std::cerr << "Failed to Read Current Firmware" << std::endl;
        return status;
    }

    if (out_ver) {
        memcpy(out_ver, Current_FW_Version, 44);
    }

    snprintf(version, sizeof(version), "%d.%d.%d.%d", Current_FW_Version[0],
             Current_FW_Version[1], Current_FW_Version[2],
             Current_FW_Version[3]);
    std::cout << "Current firmware version is : " << version << std::endl;

    return status;
}

bool Adsd3500::write_cmd(uint16_t cmd, uint16_t data) {
    bool status = true;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_ADSD3500_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 4;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    buf[5] = uint8_t(data >> 8);
    buf[6] = uint8_t(data & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "Writing Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        close(debug_fd);
        close(sfd);
        exit(EXIT_FAILURE);
    }
    return true;
}

bool Adsd3500::write_payload(uint8_t *payload, uint16_t payload_len) {
    bool status = true;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_ADSD3500_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    if (payload_len > (ADSD3500_CTRL_PACKET_SIZE - 3)) {
        std::cerr << "write_payload: payload_len " << payload_len
                  << " exceeds buffer capacity" << std::endl;
        return false;
    }
    memcpy(buf + 3, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        std::cout << "Writing Adsd3500 error "
                  << "errno: " << errno << " error: " << strerror(errno)
                  << std::endl;
        return false;
    }

    usleep(100 * 1000);

    return status;
}

bool Adsd3500::read_cmd(uint16_t cmd, uint16_t *data) {
    if (!data) {
        std::cerr << "read_cmd: null data pointer" << std::endl;
        return false;
    }
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_ADSD3500_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 2;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "0. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    // wait for the last frame processing time, needed for adsd3500
    usleep(double(1.0) / 30 * 1000000);

    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "1. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    if (xioctl(sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "2. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    *data = (uint16_t)(extCtrl.p_u8[3] << 8) + (uint16_t)(extCtrl.p_u8[4]);
    return true;
}

bool Adsd3500::read_burst_cmd(uint8_t *payload, uint16_t payload_len,
                              uint8_t *data) {
    if (!data) {
        std::cerr << "read_burst_cmd: null data pointer" << std::endl;
        return false;
    }
    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_ADSD3500_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    if (payload_len > (ADSD3500_CTRL_PACKET_SIZE - 3)) {
        std::cerr << "read_burst_cmd: payload_len " << payload_len
                  << " exceeds buffer capacity" << std::endl;
        return false;
    }
    memcpy(buf + 3, payload, payload_len);
    extCtrl.p_u8 = buf;

    std::cout << std::dec;
    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "0. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    buf[0] = 0;
    buf[1] = buf[4];
    buf[2] = buf[5];

    extCtrl.p_u8 = buf;

    // wait for the last frame processing time, needed for adsd3500
    usleep(double(1.0) / 30 * 1000000);

    if (xioctl(sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "1. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    usleep(double(1.0) / 30 * 1000000);
    if (xioctl(sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        fprintf(stderr, "2. Reading Adsd3500: %d error: %s\n", errno,
                strerror(errno));
        return false;
    }

    for (int i = 0; i < 44; i++) {
        data[i] = extCtrl.p_u8[i + 3];
    }
    printf("\n");
    return true;
}
