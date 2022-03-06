#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdint>
#include <string>
#include "CRC/include/compute_crc.h"

/* Seed value for CRC computation */
#define ADI_ROM_CFG_CRC_SEED_VALUE                      (0xFFFFFFFFu)

/* CRC32 Polynomial to be used for CRC computation */
#define ADI_ROM_CFG_CRC_POLYNOMIAL                      (0x04C11DB7u)

// standard read
uint16_t pulsatrix_read_cmd(uint16_t cmd); 
// standard write
void pulsatrix_write_cmd(uint16_t cmd, uint16_t data); 
// burst write, used to send out command
void pulsatrix_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len);  
// burst write, used to send out payload and send fw upgrade cmd header
void pulsatrix_write_payload(uint8_t* payload, uint16_t payload_len);  
// burst read 'payload_len' bytes data
void pulsatrix_read_payload_cmd(uint32_t cmd, uint16_t payload_len, uint8_t* readback_data);

typedef union
{
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

uint32_t nResidualCRC = ADI_ROM_CFG_CRC_SEED_VALUE;

int pulsa_fw_upgrade(std::string filepath)
{
    // Read Chip ID in STANDARD mode
    uint16_t chip_id = pulsatrix_read_cmd(0x0112);
    std::cout << "The readback chip ID is: " << std::hex << chip_id << std::endl;

    // Switch to BURST mode.
    pulsatrix_write_cmd(0x0019, 0x0000);

    // Send FW content, each chunk is 256 bytes
    const int flashPageSize = 256;
    int packetStart = 0;
    int packetEnd = flashPageSize;
    
    // Read the firmware binary file
    // filepath = "../fw_bin/pulsatrix_application_v3.0.0_Without_Imager.stream_3_3_3.bin";
    std::ifstream fw_file(filepath, std::ios::binary);
    // copy all data into buffer
    std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(fw_file), {});

    uint32_t fw_upgrade_cmd = 0x00160100;
    uint32_t fw_len = buffer.size();
    uint8_t* fw_content = buffer.data();
    cmd_header_t fw_upgrade_header;
    fw_upgrade_header.id8 = 0xAD;
    fw_upgrade_header.chunk_size16 = 0x0100;
    fw_upgrade_header.cmd8 = 0x04;
    fw_upgrade_header.total_size_fw32 = fw_len;
    fw_upgrade_header.header_checksum32 = (0x0100 + 0x4 + fw_len);

    crc_parameters_t crc_params;
    crc_params.type = CRC_32bit;
    crc_params.polynomial.polynomial_crc32_bit = ADI_ROM_CFG_CRC_POLYNOMIAL;
    crc_params.initial_crc.crc_32bit = nResidualCRC;
    crc_params.crc_compute_flags = IS_CRC_MIRROR;

    crc_output_t res = compute_crc(&crc_params, fw_content, fw_len);
    nResidualCRC = res.crc_32bit;

    fw_upgrade_header.crc_of_fw32 = nResidualCRC;

    pulsatrix_write_payload(fw_upgrade_header.cmd_header_byte, fw_len);

    int packetsToSend;
    if ((fw_len % flashPageSize) != 0) {
        packetsToSend = (fw_len/flashPageSize + 1);
    }
    else {
        packetsToSend = (fw_len/flashPageSize);
    }

    uint8_t data_out[flashPageSize];

    for (int i=0; i<packetsToSend; i++) {
        int start = flashPageSize * i;
        int end = flashPageSize * (i+1);
        
        for (int j=start; j<end; j++){
            if (j < fw_len) {
                data_out[j-start] = fw_content[j];
            }
            else {
                // padding with 0x00
                data_out[j-start] = 0x00;
            }
        }
        pulsatrix_write_payload(data_out, flashPageSize);
    }

    return 0;
}
