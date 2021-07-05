/*
MIT License

Copyright (c) 2021 Analog Devices, Inc.

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
*/
#include "calibration_itof.h"

#include <aditof/depth_sensor_interface.h>

#include <glog/logging.h>

using namespace aditof;

namespace adsd3100 {

/**
 * @enum Integer defines used internally to configure the FPGA registers, power monitor registers, UVC driver, camera registers, AFE uSeq indirect access
 * @brief Maintains the definition of FPGA registers, power monitor registers, UVC driver, camera registers, AFE uSeq indirect access
 */
enum : uint16_t {

    CHIPID = 0x5931,

    // FIXME: extract header definitions to another class
    EMBED_HDR_LENGTH = 128,
    SUBFRAMES_HEADER_LENGTH = 64,
    REG_CAPTURE_ID_LOC = 49,
    REG_MODE_ID_CURR_LOC = 48,
    FRAME_WIDTH_LOC = 12,
    FRAME_HEIGHT_LOC = 14,
    FRAME_NUM_LOC = 8,
    CHIPID_LOC = 0,
    TEMP_SENSOR_ADC = 28,
    CAPTURE_START_TIME = 24,
    CAPTURE_END_TIME = 26,

    MAX_NUMBER_MODES = 16,

    /* uSeq RAM indirect access */
    USEQ_RAM_LOAD_ADDR_REG_ADDR = 0x0000,
    USEQ_RAM_LOAD_DATA_REG_ADDR = 0x0002,
    USEQ_RAM_RDST_ADDR_REG_ADDR = 0x0004,
    USEQ_RAM_RD_DATA_REG_ADDR = 0x0006,

    /* USEQ Control   */
    USEQ_CONTROL_REG_ADDR = 0x000C,
    USEQ_CONTROL_START_CMD = 0x00C5,
    USEQ_CONTROL_SHUTDOWN_CMD = 0x0002,
    USEQ_CONTROL_START_CMD_DATA = 1,
    USEQ_CONTROL_STOP_CMD_DATA = 0,

    USEQ_CONTROL_STATUS_REG_ADDR = 0x0256,
    USEQ_CONTROL_ERROR_STATUS_REG_ADDR = 0x0032,

    DIG_PWR_DOWN_REG_ADDR = 0x0014,
    DIG_PWR_DOWN_OPERATING_CMD = 0x0000,
    DIG_PWR_DOWN_CLOCK_GATED_CMD = 0x0014,

    /* Global ADC Settings */
    ADC_CTRL1_S1_REG_ADDR = 0x0102,
    ADC_CTRL2_S1_REG_ADDR = 0x0104,

    /* Global Gain Comparator */
    DAC_CTRL1_REG_ADDR = 0x0126,
    DAC_CTRL2_S1_REG_ADDR = 0x012e,
    DAC_CTRL3_S1_REG_ADDR = 0x0130,
    DAC_CTRL3_S1_REF_DAC4_DEFAULT = 0x1f00,
    DAC_DATA_REG_ADDR = 0x0132,

    /* comp_ref_dac4 */
    COMP_REF_DAC4_DEFAULT = 0x1f00,

    /* Scratchpad Register (mode) */
    SET_MODE_REG_ADDR = 0x0200,

    /* data RAM indirect access */
    IA_SELECT_REG_ADDR = 0x0500,
    IA_ADDR_REG_REG_ADDR = 0x0502,
    IA_WRDATA_REG_REG_ADDR = 0x0504,
    IA_SELECT_GAINMEM_ENABLE = 0x0008,
    IA_SELECT_OFFSETMEM_ENABLE = 0x0004,

    /* VLow reg */
    VLOW_REG = 0x0172,

    /* REG_8b_PCM_GAIN_SEL_BLACK_ADJ */
    REG_8b_PCM_GAIN_SEL_BLACK_ADJ = 0x0246,

    /* Global ADC Gain   */
    INV_ADC_GAIN_0_REG_ADDR = 0x0520,
    INV_ADC_GAIN_1_REG_ADDR = 0x0522,
    INV_ADC_GAIN_2_REG_ADDR = 0x0524,
    INV_ADC_GAIN_3_REG_ADDR = 0x0526,

    TIE_PC_BYPASS_REG_ADDR = 0x0528,
    TIE_PC_BYPASS_ENBL_GAIN = 0x000A,
    TIE_PC_BYPASS_ENBL_GAINOFFSET = 0x0008,

    /* LSMOD */
    LSCTRL0_S1_REG_ADDR = 0x0138,
    LSCTRL0_S1_DEFAULT = 0x0089,
};

} //namespace adsd3100

/*
 * Constructor expects an instance of depth sensor that is already opened for communication.
 */
CalibrationItof::CalibrationItof(std::shared_ptr<DepthSensorInterface> sensor)
    : m_sensor(sensor) {}

Status
CalibrationItof::writeConfiguration(const std::string &configurationFile) {
    Status status = Status::OK;

    FILE *fd;
    fopen_s(&fd, (const char *)configurationFile.c_str(), "rb");
    if (!fd) {
        LOG(WARNING) << "Cannot open sensor configuration file: "
                     << configurationFile;
        return Status::GENERIC_ERROR;
    }

    m_configuration.parseConfigFile((const char *)configurationFile.c_str());

    uint32_t offset;
    m_configuration.getConfigOffset(offset_type_register, &offset);
    fseek(fd, offset, SEEK_SET);
    writeConfigBlock(fd);

    // disabled all digital clock gating before writing to RAMs
    unsigned short regAddr = adsd3100::DIG_PWR_DOWN_CLOCK_GATED_CMD;
    uint16_t regdata = adsd3100::DIG_PWR_DOWN_OPERATING_CMD;
    m_sensor->writeRegisters(&regAddr, &regdata, 1);

    const int num_write_blocks = 9;
    offset_type_enum block_writes[num_write_blocks] = {
        offset_type_lx5_ram,        offset_type_lx5_dram_bank0,
        offset_type_lx5_dram_bank1, offset_type_lx5_dram_bank2,
        offset_type_lx5_dram_bank3, offset_type_seqram,
        offset_type_mapram,         offset_type_wavram,
        offset_type_misc_register,
    };

    for (int i = 0; i < num_write_blocks; ++i) {
        m_configuration.getConfigOffset(block_writes[i], &offset);
        fseek(fd, offset, SEEK_SET);
        writeConfigBlock(fd);
    }

    fclose(fd);
    return status;
}

Status CalibrationItof::writeCalibration(const std::string &calibrationFile) {
    // TO DO: implement this
    return Status::OK;
}

Status CalibrationItof::writeSettings(
    const std::vector<std::pair<std::string, int32_t>> &settings) {
    using namespace aditof;
    Status status = Status::OK; 

    for (int i = 0; i < settings.size(); i++) {

        if (settings[i].first == "LSMOD1_CONFIG_VALUE") {
            uint16_t ldmod1_config = settings[i].second;
            // invert SPI chip select polarity for Walden R1 modules
            if (ldmod1_config != adsd3100::LSCTRL0_S1_DEFAULT) {
            
            //Must be modified when writeRegisters wrapper function will be implemented.
                uint16_t passedValue = adsd3100::LSCTRL0_S1_REG_ADDR;
                status = m_sensor->writeRegisters(&passedValue, &ldmod1_config, 1);
                LOG(INFO) << "Overridding LSMOD1 value in Light Source Controls register!";
                if (status != Status::OK){
                    break;
                }
            }
        }
    }
    
    return status;
}

Status CalibrationItof::writeDefaultCalibration() {
    // TO DO: implement this
    return Status::OK;
}

Status CalibrationItof::writeConfigBlock(FILE *fd) {
    using namespace aditof;
    Status status = Status::OK;

    config_block config_block;

    /* Read blocks*/
    fread(&config_block.BlockID, UINT_16_BYTES, 1u, fd);
    fread(&config_block.BlockVer, UINT_16_BYTES, 1u, fd);
    fread(&config_block.BlockSize, UINT_32_BYTES, 1u, fd);
    fread(&config_block.blsBurstLayout, UINT_16_BYTES, 1u, fd);
    fread(&config_block.nBurstSetupWrites, UINT_16_BYTES, 1u, fd);
    fread(config_block.BurstSetupWrites, UINT_16_BYTES, 4u, fd);

    if (config_block.nBurstSetupWrites > 0) {

        uint16_t *burstAddr = (uint16_t *)malloc(
            sizeof(uint16_t) * config_block.nBurstSetupWrites);
        uint16_t *burstData = (uint16_t *)malloc(
            sizeof(uint16_t) * config_block.nBurstSetupWrites);
        for (uint32_t i = 0u; i < config_block.nBurstSetupWrites; i++) {
            // fprintf(fp, "0x%4.4x, 0x%4.4x\n", config_block.BurstSetupWrites[2*i], config_block.BurstSetupWrites[2*i+1]);
            burstAddr[i] = config_block.BurstSetupWrites[2 * i];
            burstData[i] = config_block.BurstSetupWrites[2 * i + 1];
        }

        status = m_sensor->writeRegisters(burstAddr, burstData,
                                          config_block.nBurstSetupWrites);

        if (status != Status::OK) {
            LOG(WARNING) << "Error writting setup registers to imager sensor";
        }

        free(burstAddr);
        free(burstData);
    }

    fread(&config_block.startAddress, UINT_16_BYTES, 1u, fd);
    fread(&config_block.rsvd, UINT_16_BYTES, 1u, fd);
    fread(&config_block.nValues, UINT_32_BYTES, 1u, fd);

    if (config_block.blsBurstLayout == 1) {

        /* Burst Write */
        uint8_t *Data =
            (uint8_t *)malloc(config_block.nValues * sizeof(uint16_t));

        for (uint32_t i = 0u; i < config_block.nValues; i++) {

            uint16_t aux = 0;
            if (fread(&aux, UINT_16_BYTES, 1, fd) != 1) {
                LOG(WARNING) << "Error reading cfg file";
            }
            //byte swap for burst write
            Data[2 * i] = aux >> 8;
            Data[2 * i + 1] = aux & 0xFF;
            aux = Data[2 * i] << 8 | Data[2 * i + 1];
        }

        status =
            m_sensor->writeRegisters(&config_block.startAddress,
                                     reinterpret_cast<const uint16_t *>(Data),
                                     config_block.nValues, true);
        if (status != Status::OK) {
            LOG(WARNING)
                << "Error writting indirect registers to imager sensor";
        }
        free(Data);

    } else {

        /* Address data pairs */
        uint16_t numDataPairs = config_block.nValues / 2;
        uint16_t *data = (uint16_t *)malloc(sizeof(uint16_t) * numDataPairs);
        uint16_t *addr = (uint16_t *)malloc(sizeof(uint16_t) * numDataPairs);
        for (uint32_t i = 0u; i < numDataPairs; i++) {

            uint16_t nAddr;
            uint16_t nData;
            if (fread(&nAddr, sizeof(uint16_t), 1u, fd) != 1) {
                LOG(WARNING) << "Error reading cfg file";
            }

            if (fread(&nData, sizeof(uint16_t), 1u, fd) != 1) {
                LOG(WARNING) << "Error reading cfg file";
            }

            addr[i] = nAddr;
            data[i] = nData;
        }

        status = m_sensor->writeRegisters(addr, data, numDataPairs);
        if (status != Status::OK) {
            LOG(WARNING) << "Error writting registers to imager sensor";
        }

        free(data);
        free(addr);
    }

    return status;
}