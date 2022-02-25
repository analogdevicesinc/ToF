#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdint>
#include <string>

// standard read
uint16_t pulsatrix_read_cmd(uint16_t cmd); 
// standard write
void pulsatrix_write_cmd(uint16_t cmd, uint16_t data); 
// burst write, used to send out command
void pulsatrix_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len);  
// burst write, used to send out payload
void pulsatrix_write_payload(uint8_t* payload, uint16_t payload_len);  
// burst read 'payload_len' bytes data
void pulsatrix_read_payload_cmd(uint32_t cmd, uint16_t payload_len, uint8_t* readback_data);

int pulsa_fw_upgrade(std::string filepath)
{
    // Read Chip ID in STANDARD mode
    uint16_t chip_id = pulsatrix_read_cmd(0x0112);
    std::cout << "The readback chip ID is: " << std::hex << chip_id << std::endl;

    // Switch to BURST mode.
    // Assuming the switching happens automatically, handled in lower level SDK
    // send_pulsatrix_cmd(0x0019, 0x0000);

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
    uint16_t fw_len = buffer.size();
    uint8_t* fw_content = buffer.data();
    pulsatrix_write_payload_cmd(fw_upgrade_cmd, fw_content, fw_len);

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
