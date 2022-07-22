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

#ifndef ADSD3100_CONFIGURATION_H
#define ADSD3100_CONFIGURATION_H

#include "aditof_common.h"
#include "configuration.h"

#include <aditof/status_definitions.h>
#include <stdint.h>

#define UINT_16_BYTES    2u
#define UINT_32_BYTES    4u
#define UINT_8_BYTES     1u

#define LCREG_LEN        8u*16u
#define POKEREG_LEN      8u*16u
#define PADREG_LEN       4u*16u

namespace aditof {

/**
 * @struct config_offset_table_struct
 * @brief Describes the properties of configuration offset table
 */
typedef struct 
{
    uint16_t BlockID;
    uint16_t BlockVer;
    uint32_t BlockSize;
    uint32_t INDIRECT_REG_NAMES_Offset;
    uint32_t LX5_IRAM_Offset;
    uint32_t LX5_DRAM_BANK0_Offset;
    uint32_t LX5_DRAM_BANK1_Offset;
    uint32_t LX5_DRAM_BANK2_Offset;
    uint32_t LX5_DRAM_BANK3_Offset;
    uint32_t REG_Offset;
    uint32_t SEQRAM_Offset;
    uint32_t MAPRAM_Offset;
    uint32_t WAVRAM_Offset;
    uint32_t MISC_REGS_Offset;
}config_offset_table_struct;

/**
 * @struct config_indirect_regs_struct
 * @brief Describes the configuration params needed for Indirect register memory access
 */
typedef struct 
{
    uint16_t BlockID;
    uint16_t BlockVer;
    uint32_t BlockSize;
    uint8_t LCRegStr[LCREG_LEN];
    uint8_t PokeRegStr[POKEREG_LEN];
    uint8_t PadRegStr[PADREG_LEN];
}config_indirect_regs_struct;

/**
 * @struct config_block
 * @brief Describes the properties of configuration block
 */
typedef struct 
{
    uint16_t BlockID;
    uint16_t BlockVer;
    uint32_t BlockSize;
    uint16_t blsBurstLayout;
    uint16_t nBurstSetupWrites;
    uint16_t BurstSetupWrites[4];
    uint16_t startAddress;
    uint16_t rsvd;
    uint32_t nValues;
    // Data follows for the size of nValues
    //uint16_t Values[nValues]
}config_block;

/**
 * @struct config_struct
 * @brief Describes the skeleton of CFG file
 */
typedef struct 
{
    file_header_struct fileHeader;
    config_offset_table_struct configOffset;
    config_indirect_regs_struct indirectRegs;

}config_struct;

/**
 * @enum offset_type_enum
 * @brief type of memory offset 
 */
typedef enum 
{
    offset_type_register,
    offset_type_seqram, 
    offset_type_mapram, 
    offset_type_wavram, 
    offset_type_lx5_ram, 
    offset_type_lx5_dram_bank0,
    offset_type_lx5_dram_bank1,
    offset_type_lx5_dram_bank2,
    offset_type_lx5_dram_bank3,
    offset_type_misc_register,
   
}offset_type_enum; 

/**
 * @class ADSD3100Configuration
 * @brief Parses the CFG file and maintains all the offset and header information in the member variable configuration
 * 
 */
class ADSD3100Configuration {
  public:
    
    /**
     * @brief Destructor
     */
    ~ADSD3100Configuration() = default;

  public:
    /**
     * @brief Returns the value stored at the particular offset_type from already configred "configuration" member variable
     * 
     * @param[in] offset_type - Type of offset value to be returned
     * @param[out] offset- Based on the offset_type, this buffer gets filled up from alrady stored value
     * @return aditof::Status
     * @see aditof::Status
     * @see offset_type_enum
     */
    aditof::Status getConfigOffset(offset_type_enum offset_type, uint32_t *offset);

    /**
     * @brief Returns the offset value stored at the particular offset_type from already configred HSP "configuration" member variable
     *
     * @param[in] offset_type - Type of offset value to be returned
     * @param[out] offset- Based on the offset_type, this buffer gets filled up from alrady stored value
     * @return aditof::Status
     * @see aditof::Status
     * @see offset_type_enum
     */
    //aditof::Status getConfigHSPOffset(offset_type_enum offset_type, uint32_t *offset);

    /**
     * @brief Parses the CFG file_name, which is an camera firmware and stores all the feilds in the member variable
     * 
     * @param[in] file_name - config.cfg file passed from the json file 
     * @return aditof::Status
     * @see aditof::Status
     * @see <a href='../sdk/config/config_default.json'>config_default.json</a>
     */
    aditof::Status parseConfigFile(const char *file_name);

  private:
    config_struct configuration;
};

} //namespace aditof

#endif /*ADSD3100_CONFIGURATION_H*/
