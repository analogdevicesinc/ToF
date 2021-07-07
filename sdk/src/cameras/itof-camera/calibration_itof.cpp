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
#include "ccb.h"

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
    writeRegister(adsd3100::DIG_PWR_DOWN_OPERATING_CMD, adsd3100::DIG_PWR_DOWN_OPERATING_CMD);

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
    Status status = Status::OK;

    // Load CCB File
    void *ccbData = 0;
    size_t size_ccb = 0;
    FILE *f = NULL;

    if (calibrationFile.empty())
    {
        //FIXME: Remove use of default settings and require a CCB file
        LOG(ERROR) << "No Calibration file defined. Default calibration written by driver.\n";
        //writeDefaultCalibration();
        return status;
    }

    int err = fopen_s(&f, calibrationFile.c_str(), "rb");
    if (f) {
        if (0 == fseek(f, 0L, SEEK_END)) {
            size_ccb = ftell(f);
            rewind(f);
            ccbData = malloc(size_ccb);
            if (ccbData) {
                if (1 != fread(ccbData, size_ccb, 1, f)) {
                    LOG(WARNING) << "Failed to read data file.\n";
                    free(ccbData);
                    ccbData = 0;
                    size_ccb = 0;
                }
            } else {
                LOG(WARNING) << "Failed to allocate memory for CCB data.\n";
            }
        } else {
            LOG(WARNING) << "Failed to seek in data file.\n";
        }
        fclose(f);
    } else {
        LOG(ERROR) << "Failed to open calibration data file: " << calibrationFile;
        return aditof::Status::GENERIC_ERROR;
    }

    ccb_data_t ccb_data = {(const unsigned char *const)ccbData, size_ccb};

    const struct CAL_ADDRVAL_REG_BLOCK_V1 *calregblock = ccb_get_cal_block_addrval_reglist(&ccb_data, 0);
    const struct CAL_GAIN_CORRECTION_BLOCK *gaincorrection = ccb_get_cal_block_gaincorrection(&ccb_data, 0);
    const struct CAL_LSDAC_BLOCK_V1 *lsdac = ccb_get_cal_block_lsdacs(&ccb_data, 0);
    if (lsdac) {
        // lsdac table
        uint16_t startAddress = lsdac->StartAddress;
        writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, 0x1d29); // lsdac checksum MSB
        writeRegister(adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR, lsdac->ChecksumMSB);
        writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, 0x1d25); // lsdac checksum LSB
        writeRegister(adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR, lsdac->ChecksumLSB);
        writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, startAddress);

        uint16_t *buff = (uint16_t *)malloc(lsdac->nWrites * 2);
        memcpy(buff, lsdac->Settings, lsdac->nWrites * 2);
        for (uint32_t i = 0u; i < lsdac->nWrites; i++) {
            buff[i] = (lsdac->Settings[i] >> 8) | (0xFF00 & (lsdac->Settings[i] << 8)); // endian byte swap
        }
        uint16_t regAddr = adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR;
        m_sensor->writeRegisters(&regAddr, buff, lsdac->nWrites, true);
        free(buff);
    }

    if (calregblock && gaincorrection) {
        // pixel dac trim registers
        for (int i = 0; i < calregblock->nTuples; ++i) {
            writeRegister(calregblock->Tuples[i].Address, calregblock->Tuples[i].Value);
        }

        // global ADC settings
        writeRegister(adsd3100::ADC_CTRL1_S1_REG_ADDR, gaincorrection->ADC.UpdnoOffset);
        writeRegister(adsd3100::ADC_CTRL2_S1_REG_ADDR, gaincorrection->ADC.IRamp);

        // gain comparator
        uint16_t dacCtrlValue = gaincorrection->GainComparator.Vref2Dac;
        dacCtrlValue = (dacCtrlValue << 8) | gaincorrection->GainComparator.Vref1Dac;
        writeRegister(adsd3100::DAC_CTRL2_S1_REG_ADDR, dacCtrlValue);

        dacCtrlValue = adsd3100::COMP_REF_DAC4_DEFAULT; // comp_ref_dac4 default
        dacCtrlValue |= gaincorrection->GainComparator.Vref3Dac;
        writeRegister(adsd3100::DAC_CTRL3_S1_REG_ADDR, dacCtrlValue);

        // global gain scale registers
        writeRegister(adsd3100::INV_ADC_GAIN_0_REG_ADDR, gaincorrection->InverseGlobalADCGain[0]);
        writeRegister(adsd3100::INV_ADC_GAIN_1_REG_ADDR, gaincorrection->InverseGlobalADCGain[1]);
        writeRegister(adsd3100::INV_ADC_GAIN_2_REG_ADDR, gaincorrection->InverseGlobalADCGain[2]);
        writeRegister(adsd3100::INV_ADC_GAIN_3_REG_ADDR, gaincorrection->InverseGlobalADCGain[3]);

        const COLUMN_GAIN_CORRECTION *gaintable = &(gaincorrection->PerColGainAdjustment);
        std::vector<uint16_t> gainbuffer(sizeof(gaincorrection->PerColGainAdjustment) / 2);
        memcpy(gainbuffer.data(), (const uint16_t *)gaintable, sizeof(uint16_t) * (sizeof(gaincorrection->PerColGainAdjustment) / 2));

        writeRegister(adsd3100::DIG_PWR_DOWN_REG_ADDR, adsd3100::DIG_PWR_DOWN_OPERATING_CMD);
        writeRegister(adsd3100::IA_SELECT_REG_ADDR, 0x0008);
        writeRegister(adsd3100::IA_ADDR_REG_REG_ADDR, 0x0000);

        uint16_t regAddr = adsd3100::IA_WRDATA_REG_REG_ADDR;
        m_sensor->writeRegisters(&regAddr, gainbuffer.data(), (sizeof(gaincorrection->PerColGainAdjustment) / 2), true);

        // enable gain
        writeRegister(adsd3100::TIE_PC_BYPASS_REG_ADDR, adsd3100::TIE_PC_BYPASS_ENBL_GAIN);

        const COLUMN_GAIN_CORRECTION *offsettable = &(gaincorrection->PerColOffsetAdjustment);

        std::vector<uint16_t> offsetbuffer(sizeof(gaincorrection->PerColOffsetAdjustment) / 2);
        memcpy(offsetbuffer.data(), (const uint16_t *)offsettable, sizeof(uint16_t) * (sizeof(gaincorrection->PerColOffsetAdjustment) / 2));

        writeRegister(adsd3100::DIG_PWR_DOWN_REG_ADDR, adsd3100::DIG_PWR_DOWN_OPERATING_CMD);

        writeRegister(adsd3100::IA_SELECT_REG_ADDR, 0x0004);
        writeRegister(adsd3100::IA_ADDR_REG_REG_ADDR, 0x0000);

        regAddr = adsd3100::IA_WRDATA_REG_REG_ADDR;
        m_sensor->writeRegisters(&regAddr, offsetbuffer.data(), (sizeof(gaincorrection->PerColOffsetAdjustment) / 2), true);

        // enable offset
        writeRegister(adsd3100::TIE_PC_BYPASS_REG_ADDR, adsd3100::TIE_PC_BYPASS_ENBL_GAINOFFSET);
    } else {
        LOG(WARNING) << "Calibration file error\n";
        status = aditof::Status::GENERIC_ERROR;
    }

    free(ccbData);
    return status;
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
                writeRegister(adsd3100::LSCTRL0_S1_REG_ADDR, ldmod1_config);
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
    /* Write default analog registers */
    const int lsdac_blocksize = 33 + 2;                 //(usecases*freqs) + 2_word_header)
    uint16_t wav_ram_lsdac[lsdac_blocksize] = {0x0069,  // freq 3
                                               0x0060,  // freq 2
                                               0x0061,  // freq 1
                                               0x0307,  // Mode 11 LSDAC Header (3 freq) - MP Mode
                                               0x0069,  // freq 1
                                               0x0105,  // Mode 10
                                               0x0000,  // Mode 9
                                               0x0069,  // freq 3
                                               0x0060,  // freq 2
                                               0x0061,  // freq 1
                                               0x0307,  // Mode 8
                                               0x0000,  // freq 3
                                               0x0000,  // freq 2
                                               0x0000,  // freq 1
                                               0x0303,  // Mode 7 LSDAC Header (3 freq)
                                               0x0069,  // freq 3
                                               0x0060,  // freq 2
                                               0x0061,  // freq 1
                                               0x0307,  // Mode 6 LSDAC Header (3 freq) - MP Mode
                                               0x0060,  // freq 3
                                               0x0059,  // freq 2
                                               0x0059,  // freq 1
                                               0x0301,  // Mode 5 LSDAC Header (3 frea)
                                               0x0000,  // Mode 4 LSDAC Header (0 freq)
                                               0x006f,  // freq 1
                                               0x0105,  // Mode 3 LSDAC Header (1 freq)
                                               0x006e,  // freq 3
                                               0x0067,  // freq 2
                                               0x0063,  // freq 1
                                               0x0301,  // Mode 2 LSDAC header (3 freq)
                                               0x0068,  // freq 2
                                               0x006e,  // freq 1
                                               0x0205,  // Mode 1 LSDAC header (2 freq)
                                               0x646b,  // structural checksum
                                               0x0021}; // lsdac table size

    // wave RAM, LSADC calibration settings
    writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, 0x1d29); // lsdac checksum MSB
    writeRegister(adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR, 0x43D1);
    writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, 0x1d25); // lsdac checksum LSB
    writeRegister(adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR, 0x0000);
    writeRegister(adsd3100::USEQ_RAM_LOAD_ADDR_REG_ADDR, 0x1c99);

    uint16_t startAddress = adsd3100::USEQ_RAM_LOAD_DATA_REG_ADDR;
    //m_sensor->writeRegisters(&startAddress, wav_ram_lsdac, lsdac_blocksize, true);

    // vlowreg
    writeRegister(adsd3100::VLOW_REG, 0x0a0a);

    // DAC data configuraiton
    writeRegister(adsd3100::DAC_DATA_REG_ADDR, 0x0047);
    writeRegister(adsd3100::DAC_CTRL1_REG_ADDR, 0x0020);
    writeRegister(adsd3100::DAC_DATA_REG_ADDR, 0x0088);
    writeRegister(adsd3100::DAC_CTRL1_REG_ADDR, 0x0001);
    writeRegister(adsd3100::DAC_DATA_REG_ADDR, 0x001d);
    writeRegister(adsd3100::DAC_CTRL1_REG_ADDR, 0x0008);
    writeRegister(adsd3100::DAC_DATA_REG_ADDR, 0x008b);
    writeRegister(adsd3100::DAC_CTRL1_REG_ADDR, 0x0010);
    writeRegister(adsd3100::DAC_DATA_REG_ADDR, 0x0075);
    writeRegister(adsd3100::DAC_CTRL1_REG_ADDR, 0x0040);

    // REG_8b_PCM_GAIN_SEL_BLACK_ADJ (doen't seem to be used in uSeq)
    writeRegister(adsd3100::REG_8b_PCM_GAIN_SEL_BLACK_ADJ, 0xc1fa);

    // GLOBAL_ADC_SETTINGS
    writeRegister(adsd3100::ADC_CTRL1_S1_REG_ADDR, 0x019e); // adc_ctrl1_s1
    writeRegister(adsd3100::ADC_CTRL2_S1_REG_ADDR, 0x009a); // adc_ctrl2_s1

    writeRegister(adsd3100::DAC_CTRL2_S1_REG_ADDR, 0x060a); // dac_ctrl2_s1
    writeRegister(adsd3100::DAC_CTRL3_S1_REG_ADDR, 0x1f10); // dac_ctrl3_s1

    // Write inv_adc_gain_x
    writeRegister(adsd3100::INV_ADC_GAIN_0_REG_ADDR, 0x1a00); // inv_adc_gain_0
    writeRegister(adsd3100::INV_ADC_GAIN_1_REG_ADDR, 0x1627); // inv_adc_gain_1
    writeRegister(adsd3100::INV_ADC_GAIN_2_REG_ADDR, 0x1321); // inv_adc_gain_2
    writeRegister(adsd3100::INV_ADC_GAIN_3_REG_ADDR, 0x0a8a); // inv_adc_gain_3
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

Status CalibrationItof::writeRegister(uint16_t reg_address, uint16_t reg_data) {
    return CalibrationItof::m_sensor->writeRegisters(&reg_address, &reg_data, 1);
}
