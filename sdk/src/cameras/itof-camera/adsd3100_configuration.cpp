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

#include "ADSD3100_configuration.h"

#include <stdio.h>

namespace aditof {

aditof::Status ADSD3100Configuration::getConfigOffset(offset_type_enum offset_type, uint32_t *offset) {
    using namespace aditof;

    if (offset_type == offset_type_register) {
        *offset = configuration.configOffset.REG_Offset;
    } else if (offset_type == offset_type_seqram) {
        *offset = configuration.configOffset.SEQRAM_Offset;
    }else if (offset_type == offset_type_mapram) {
        *offset = configuration.configOffset.MAPRAM_Offset;
    }else if (offset_type == offset_type_wavram) {
        *offset = configuration.configOffset.WAVRAM_Offset;
    }else if (offset_type == offset_type_lx5_ram) {
        *offset = configuration.configOffset.LX5_IRAM_Offset;
    }else if (offset_type == offset_type_lx5_dram_bank0) {
        *offset = configuration.configOffset.LX5_DRAM_BANK0_Offset;
    }else if (offset_type == offset_type_lx5_dram_bank1) {
        *offset = configuration.configOffset.LX5_DRAM_BANK1_Offset;
    }else if (offset_type == offset_type_lx5_dram_bank2) {
        *offset = configuration.configOffset.LX5_DRAM_BANK2_Offset;
    }else if (offset_type == offset_type_lx5_dram_bank3) {
        *offset = configuration.configOffset.LX5_DRAM_BANK3_Offset;
    }else if (offset_type == offset_type_misc_register) {
        *offset = configuration.configOffset.MISC_REGS_Offset;
    }

    return Status::OK;
}


aditof::Status ADSD3100Configuration::parseConfigFile(const char *file_name) {
    using namespace aditof;
    FILE *fid;

    /* Open configuration file */
    fopen_s(&fid, file_name, "rb");

    /* Read file signature */
    fread(&configuration.fileHeader.FileSignature, sizeof(uint8_t), 4u, fid);
    
    /* Read file header */
    fread(&configuration.fileHeader.header.FileFormatVersion, sizeof(uint16_t), 1u, fid);
    fread(&configuration.fileHeader.header.ConfigFileHdrSize, sizeof(uint16_t), 1u, fid);
    fread(&configuration.fileHeader.header.ConfigFileSize, sizeof(uint32_t), 1u, fid);
    fread(&configuration.fileHeader.header.nBlocks, sizeof(uint32_t), 1u, fid);
    fread(&configuration.fileHeader.header.ConfigVerMajor, sizeof(uint16_t), 1u, fid);
    fread(&configuration.fileHeader.header.ConfigVerMinor, sizeof(uint16_t), 1u, fid);
    fread(&configuration.fileHeader.header.ChipId, sizeof(uint16_t), 1u, fid);
    fread(&configuration.fileHeader.header.SystemId, sizeof(uint16_t), 1u, fid);
    
    /* Read offset table */
    fread(&configuration.configOffset.BlockID, sizeof(uint16_t), 1u, fid);
    fread(&configuration.configOffset.BlockVer, sizeof(uint16_t), 1u, fid);
    fread(&configuration.configOffset.BlockSize, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.INDIRECT_REG_NAMES_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.LX5_IRAM_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.LX5_DRAM_BANK0_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.LX5_DRAM_BANK1_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.LX5_DRAM_BANK2_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.LX5_DRAM_BANK3_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.REG_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.SEQRAM_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.MAPRAM_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.WAVRAM_Offset, sizeof(uint32_t), 1u, fid);
    fread(&configuration.configOffset.MISC_REGS_Offset, sizeof(uint32_t), 1u, fid);

    fclose(fid);
    return Status::OK;
}

} //namespace aditof