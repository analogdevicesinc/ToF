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
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ccb.h>
#include <glog/logging.h>

#include "configuration.h"
#include "aditof_common.h"
#include "temporary_filename.h"
#include "module_memory.h"
#include "crc.h"

namespace aditof {

std::string escapePathBackslash(const std::string &path) {
    std::string pathstring = path;
    std::string::size_type index = -4;
    // escape path string backslashes for JSON
    while (std::string::npos != (index = pathstring.find("\\", index + 4))) {
        pathstring.insert(index, "\\");
    }
	return pathstring;
}

bool ModuleMemory::verifyChunkHeader(const TOF_ChunkHeader_t *const pChunkHeader) {

    if (NULL == pChunkHeader) {
        return (false);
    }

    LOG(INFO) << "Reading chunk id: " << std::hex << (int)pChunkHeader->revision.chunkid << (int)pChunkHeader->revision.chunktype;
    LOG(INFO) << "Reading chunk revision: " << (int)pChunkHeader->revision.major << "." << (int)pChunkHeader->revision.minor;

    // version 1.0 chunk header
    if (pChunkHeader->revision.major == 1 && pChunkHeader->revision.minor == 0) {

        if (pChunkHeader->headerSizeBytes != sizeof(TOF_ChunkHeader_t)) {
            LOG(WARNING) << "Module data header invalid size";
            return (false);
        }
        // Check chunk header CRC
        if (crcFast(reinterpret_cast<const uint8_t *>(pChunkHeader), (pChunkHeader->headerSizeBytes - sizeof(uint32_t))) != // Hdr size - CRC in bytes
            pChunkHeader->chunkHeaderCRC) {
            LOG(WARNING) << "Module data header corrupted";
            return (false);
        }
    } else {
        LOG(WARNING) << "Unexpected module data version";
        return (false);
    }

    return (true);
}

Status ModuleMemory::readChunkHeader( uint8_t *const buffer, int32_t chunkDataAddr) {

    Status status = m_eeprom->read(chunkDataAddr, buffer, sizeof(TOF_ChunkHeader_t));
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read module memory";
        return Status::GENERIC_ERROR;
    }

    TOF_ChunkHeader_t *pChunkHeader = reinterpret_cast<TOF_ChunkHeader_t *>(buffer);
    if (!verifyChunkHeader(pChunkHeader)) {
        LOG(WARNING) << "Invalid module memory chunk header";
        return Status::GENERIC_ERROR;
    }

    return Status::OK;
}

std::string ModuleMemory::writeTempJSON(const std::string ccbFilename, const std::string cfgFilename) {

    std::string tempFileName = getTempFilename(std::string("adi_tof_cfgjson"));

    FILE *file = NULL;
    fopen_s(&file, tempFileName.c_str(), "w");
    if (file == NULL) {
        LOG(WARNING) << "Failed to open temporary file for writing";
        return std::string();
    }

    std::stringstream ss;
    ss << "{" << std::endl;
    if (!ccbFilename.empty()) {
        ss << "\"sensorFirmware\": \"" << escapePathBackslash(cfgFilename) << "\"";
    }
    if (!cfgFilename.empty()) {
        if (!ccbFilename.empty()) {
            ss << "," << std::endl;
        }
        ss << "\"CCB_Calibration\": \"" << escapePathBackslash(ccbFilename) << "\"";
    }
    ss << std::endl << "}" << std::endl;

    LOG(INFO) << "Writing temporary file: " << tempFileName;
    fputs(ss.str().c_str(), file);
    fclose(file);
    return tempFileName;
}

std::string ModuleMemory::writeToTempFile(const uint8_t *const dataBuffer, const uint32_t length, std::string fileTag) {

    std::string tempFileName = getTempFilename(std::string("adi_tof_") + fileTag);

    FILE *file = NULL;
    fopen_s(&file, tempFileName.c_str(), "wb");
    if (file == NULL) {
        LOG(WARNING) << "Failed to open temporary file for writing";
        return std::string();
    }

    LOG(INFO) << "Writing temporary file: " << tempFileName;
    fwrite(reinterpret_cast<const void *>(dataBuffer), sizeof(char), length, file);
    fclose(file);
    return tempFileName;
}

uint32_t ModuleMemory::loadfileData(const std::string filename, uint8_t **dataBufferOut) {

    FILE *file = NULL;
    uint8_t *dataBuffer = NULL;
    uint32_t size = 0;

    if (!filename.empty()) {
        int err = fopen_s(&file, filename.c_str(), "rb");
        if (file) {
            if (0 == fseek(file, 0L, SEEK_END)) {
                size = ftell(file);
                rewind(file);
                dataBuffer = static_cast<uint8_t *>(malloc(size));
                if (dataBuffer) {
                    if (1 != fread(dataBuffer, size, 1, file)) {
                        LOG(WARNING) << "Failed to read data file.\n";
                        free(dataBuffer);
                        dataBuffer = NULL;
                        size = 0;
                    }
                } else {
                    LOG(WARNING) << "Failed to allocate memory for CCB data.\n";
                }
            } else {
                LOG(WARNING) << "Failed to seek in data file.\n";
            }
            fclose(file);
        } else {
            LOG(ERROR) << "Failed to open data file.\n";
        }
    }

    *dataBufferOut = dataBuffer;
    return size;
}

std::string ModuleMemory::writeTempCCB(const uint8_t *const ccbFileData, const uint32_t length) {

    if (NULL == ccbFileData || 0 == length) {
        return (std::string());
    }

    ccb_data_t ccb_data = {(const unsigned char *const)ccbFileData, length};
    const CAL_FILE_HEADER_V1 *ccbHeader = ccb_read_header(&ccb_data);
    if (NULL == ccbHeader) {
        return std::string();
    }

    LOG(INFO) << "CCB file version: " << std::hex << std::setfill('0') << std::setw(2) << std::right << ccbHeader->CalibFileVersion;
    LOG(INFO) << "CCB calibration version: " << std::hex << std::setfill('0') << std::setw(2) << std::right << ccbHeader->CalibVersion;

    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(&ccb_data);
    if (NULL == header_block) {
        return std::string();
    }

    LOG(INFO) << "CCB file date: " << header_block->BlockInfo.CalibrationYear << "-" << header_block->BlockInfo.CalibrationMonth << "-"
              << header_block->BlockInfo.CalibrationDay;
    LOG(INFO) << "CCB chip ID: " << std::hex << header_block->ChipID;
    LOG(INFO) << "CCB serial number: " << std::hex << header_block->SerialNumber;

    m_serialNumber = header_block->SerialNumber;

    return writeToTempFile(ccbFileData, length, "ccb");
}

std::string ModuleMemory::writeTempCFG(const uint8_t *const cfgFileData, const uint32_t length) {

    if (NULL == cfgFileData || length < sizeof(ccb_data_t)) {
        return (std::string());
    }

    const file_header_struct *cfgfile = (const file_header_struct *)cfgFileData;
    std::string fileSignature(reinterpret_cast<const char *>(cfgfile->FileSignature));
    if (fileSignature != "CFG") {
        return (std::string());
    }

    LOG(INFO) << "CFG file version: " << cfgfile->header.FileFormatVersion;
    LOG(INFO) << "CFG configuration version: " << cfgfile->header.ConfigVerMajor << "." << cfgfile->header.ConfigVerMinor;
    LOG(INFO) << "CFG chip ID: " << std::hex << cfgfile->header.ChipId;
    LOG(INFO) << "CFG system ID: " << std::hex << cfgfile->header.SystemId;

    return writeToTempFile(cfgFileData, length, "cfg");
}

Status ModuleMemory::readLegacyModuleCCB( std::string &tempJsonFile, TOF_ModuleFiles_t &tempFiles, std::string &ccbSerialNumber) {
    // legacy module ccb data offset 1 sector
    uint32_t chunkDataAddr = 4096; 

    // Read file header
    uint8_t memoryBuffer[sizeof(CAL_FILE_HEADER_V1)] = {0};
    CAL_FILE_HEADER_V1 *fileheader = (CAL_FILE_HEADER_V1 *)memoryBuffer;
    Status status = m_eeprom->read(chunkDataAddr, memoryBuffer, sizeof(CAL_FILE_HEADER_V1));
    if (status != Status::OK) {
        LOG(WARNING) << "Failed to read module memory";
        return Status::GENERIC_ERROR;
    }
    
    if (fileheader->CalibFileVersion != 0x0303 || fileheader->CalibVersion != 0x0301) {
        LOG(WARNING) << "Invalid CCB header";
        return Status::GENERIC_ERROR;        
    }

    LOG(INFO) << "CCB file version: " << std::hex << std::setfill('0') << std::setw(2) << std::right << fileheader->CalibFileVersion;
    LOG(INFO) << "CCB calibration version: " << std::hex << std::setfill('0') << std::setw(2) << std::right << fileheader->CalibVersion;
    LOG(INFO) << "CCB HDR size: " << std::hex << std::setfill('0') << std::setw(2) << std::right << fileheader->CalibHdrSize;
    LOG(INFO) << "CCB File HDR size: " << std::hex << std::setfill('0') << std::setw(2) << std::right << fileheader->CalibFileHdrSize;
    LOG(INFO) << "CCB file size: " << std::hex << std::setfill('0') << std::setw(2) << std::right << fileheader->CalibFileSize;

    uint8_t *pccbdata = (uint8_t *)malloc(fileheader->CalibFileSize);
    if (0 == pccbdata) {
        LOG(WARNING) << "malloc failed, out of memory?";
        return Status::GENERIC_ERROR;
    }

    status = m_eeprom->read(chunkDataAddr, pccbdata, fileheader->CalibFileSize);
    if (status == Status::OK) {
        std::string ccbFilename = writeTempCCB(pccbdata, fileheader->CalibFileSize);
        if (!ccbFilename.empty()) {
            // Create temporary json configuration file
            tempJsonFile = writeTempJSON(ccbFilename, "");
            tempFiles = {};
            tempFiles.jsonFile = tempJsonFile;
            tempFiles.ccbFile = ccbFilename;
        }
    }

    ccb_data_t ccb_data = {(const unsigned char *const)pccbdata, fileheader->CalibFileSize};
    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(&ccb_data);
    if (NULL != header_block) {
        LOG(INFO) << "CCB file date: " << header_block->BlockInfo.CalibrationYear << "-" << header_block->BlockInfo.CalibrationMonth << "-"
                    << header_block->BlockInfo.CalibrationDay;
        LOG(INFO) << "CCB chip ID: " << std::hex << header_block->ChipID;
        LOG(INFO) << "CCB serial number: " << std::hex << header_block->SerialNumber;
        ccbSerialNumber = header_block->SerialNumber;
    }

    free(pccbdata);
    return status;
}

Status ModuleMemory::readModuleData( std::string &tempJsonFile, TOF_ModuleFiles_t &tempFiles) {
    // module data is always starts at address 0
    uint32_t chunkDataAddr = 0x0000;

    // Read file header
    uint8_t memoryBuffer[sizeof(TOF_ChunkHeader_t)] = {0};
    TOF_ChunkHeader_t *pChunkHeader = (TOF_ChunkHeader_t *)memoryBuffer;
    Status status = readChunkHeader(memoryBuffer, chunkDataAddr);
    if (status != Status::OK || pChunkHeader->revision.chunktype != CHUNK_TYPE_FILE_HEADER || pChunkHeader->nextChunkAddress == 0) {
        LOG(WARNING) << "Invalid ADI module memory file header";
        return Status::GENERIC_ERROR;
    }

    bool isBadChunk = false;
    std::string ccbFilename;
    std::string cfgFilename;
    std::string jsonFilename;

    // Read and parse data chunks
    while (!isBadChunk && pChunkHeader->nextChunkAddress != 0) {

        chunkDataAddr = pChunkHeader->nextChunkAddress;
        status = readChunkHeader(memoryBuffer, chunkDataAddr);
        if (Status::OK != status) {
            isBadChunk = true;
            break;
        }

        if (pChunkHeader->headerSizeBytes != sizeof(TOF_ChunkHeader_t) || 0 == pChunkHeader->chunkSizeBytes) {
            isBadChunk = true;
            break;
        }

        chunkDataAddr += pChunkHeader->headerSizeBytes;
        uint8_t *pChunkData = (uint8_t *)malloc(pChunkHeader->chunkSizeBytes);
        if (0 == pChunkData) {
            LOG(WARNING) << "malloc failed, out of memory?";
            break;
        }

        if (Status::OK == m_eeprom->read(chunkDataAddr, pChunkData, pChunkHeader->chunkSizeBytes)) {
            int payloadSize = pChunkHeader->chunkSizeBytes - sizeof(uint32_t); // data size - CRC in bytes
            uint32_t blockCRC = *((uint32_t *)(pChunkData + payloadSize));
            if (crcFast(pChunkData, payloadSize) == blockCRC) {

                if (pChunkHeader->revision.chunktype == CHUNK_TYPE_CCB) {
                    LOG(INFO) << "Found module memory CCB chunk, parsing...";
                    ccbFilename = writeTempCCB(pChunkData, payloadSize);
                    if (ccbFilename.empty()) {
                        LOG(WARNING) << "Invalid module memory CCB chunk";
                        isBadChunk = true;
                    }
                } else if (pChunkHeader->revision.chunktype == CHUNK_TYPE_CFG) {
                    LOG(INFO) << "Found module memory CFG chunk, parsing...";
                    cfgFilename = writeTempCFG(pChunkData, payloadSize);
                    if (cfgFilename.empty()) {
                        LOG(WARNING) << "Invalid module memory CFG chunk";
                        isBadChunk = true;
                    }
                } else {
                    LOG(WARNING) << "Unsupported module data chunk type";
                    isBadChunk = true;
                }
            } else {
                LOG(WARNING) << "Corrupted module memory data block type " << std::hex << pChunkHeader->revision.chunktype;
                isBadChunk = true;
            }
        } else {
            LOG(WARNING) << "Failed reading module memory data";
            isBadChunk = true;
        }
        free(pChunkData);
    }

    if (!isBadChunk) {
        // Create temporary json configuration file
        jsonFilename = writeTempJSON(ccbFilename, cfgFilename);
    }

    if (isBadChunk || jsonFilename.empty()) {
        LOG(WARNING) << "Failed to load module data!";
        std::remove(ccbFilename.c_str());
        std::remove(cfgFilename.c_str());
        return Status::GENERIC_ERROR;
    }

    tempFiles = {};
    tempFiles.jsonFile = jsonFilename;
    if (!ccbFilename.empty()) {
        tempFiles.ccbFile = ccbFilename;
    }
    if (!cfgFilename.empty()) {
        tempFiles.cfgFile = cfgFilename;
    }

    tempJsonFile = jsonFilename;
    return Status::OK;
}

TOF_ChunkHeader_t ModuleMemory::initChunckHeader(TOF_ChunkType_t type, uint32_t address, uint32_t dataSize, bool isLastChunk) {

    TOF_ChunkVesion_t fileVersion = {};
    fileVersion.chunkid = CHUNK_ID_ANALOG_DEVICES;
    fileVersion.chunktype = type;
    fileVersion.major = FLASH_MAJOR_VERSION;
    fileVersion.minor = FLASH_MINOR_VERSION;

    TOF_ChunkHeader_t header = {};
    header.revision = fileVersion;
    header.headerSizeBytes = sizeof(TOF_ChunkHeader_t);
    header.chunkSizeBytes = (dataSize == 0) ? 0 : (dataSize + sizeof(header.chunkHeaderCRC));
    if (isLastChunk) {
        header.nextChunkAddress = 0;
    } else {
        header.nextChunkAddress = address + header.headerSizeBytes + header.chunkSizeBytes;
    }
    header.chunkHeaderCRC = crcFast(reinterpret_cast<const uint8_t *>(&header), sizeof(TOF_ChunkHeader_t) - sizeof(header.chunkHeaderCRC));
    return header;
}

Status ModuleMemory::createModuleImage(const uint8_t *ccbData, uint32_t ccbSize, const uint8_t *cfgData, uint32_t cfgSize) {

    if ((ccbData == NULL && cfgData == NULL) || (ccbSize == 0 && cfgSize == 0)) {
        LOG(WARNING) << "Aborting write to Module flash, no data found";
        return Status::GENERIC_ERROR;
    }

    // create memory header chunk
    TOF_ChunkHeader_t memoryHeader = initChunckHeader(CHUNK_TYPE_FILE_HEADER, 0, 0, false);
    uint32_t nextAddress = memoryHeader.nextChunkAddress;
    uint32_t imageSize = memoryHeader.headerSizeBytes;

    // create CCB header
    TOF_ChunkHeader_t ccbHeader = {0};
    uint32_t ccbCRC = 0;
    if (ccbData && 0 != ccbSize) {
        ccbHeader = initChunckHeader(CHUNK_TYPE_CCB, nextAddress, ccbSize, cfgData == NULL);
        nextAddress = ccbHeader.nextChunkAddress;
        ccbCRC = crcFast(ccbData, ccbSize);
        imageSize += ccbHeader.headerSizeBytes + ccbHeader.chunkSizeBytes;
    }

    // create CFG header
    TOF_ChunkHeader_t cfgHeader = {0};
    uint32_t cfgCRC = 0;
    if (cfgData && 0 != cfgSize) {
        cfgHeader = initChunckHeader(CHUNK_TYPE_CFG, nextAddress, cfgSize, true);
        cfgCRC = crcFast(cfgData, cfgSize);
        imageSize += cfgHeader.headerSizeBytes + cfgHeader.chunkSizeBytes;
    }

    size_t eepromCapacity;
    aditof::Status status = m_eeprom->getCapacity(eepromCapacity);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get the memory capacity!";
        return status;
    }
    if (imageSize > eepromCapacity) {
        LOG(ERROR) << "Module data size exceeds Flash memory capacity!";
        return aditof::Status::GENERIC_ERROR;
    }

    uint8_t *write_buff = (uint8_t *)malloc(imageSize);
    if (NULL == write_buff) {
        LOG(WARNING) << "malloc failed, out of memory?";
        return aditof::Status::GENERIC_ERROR;
    }

    memcpy(write_buff, &memoryHeader, memoryHeader.headerSizeBytes);
    nextAddress = memoryHeader.nextChunkAddress;
    if (ccbData) {
        memcpy(write_buff + nextAddress, &ccbHeader, ccbHeader.headerSizeBytes);
        nextAddress += ccbHeader.headerSizeBytes;
        memcpy(write_buff + nextAddress, ccbData, ccbSize);
        nextAddress += ccbSize;
        memcpy(write_buff + nextAddress, &ccbCRC, sizeof(uint32_t));
        nextAddress += sizeof(uint32_t);
    }

    if (cfgData) {
        memcpy(write_buff + nextAddress, &cfgHeader, cfgHeader.headerSizeBytes);
        nextAddress += cfgHeader.headerSizeBytes;
        memcpy(write_buff + nextAddress, cfgData, cfgSize);
        nextAddress += cfgSize;
        memcpy(write_buff + nextAddress, &cfgCRC, sizeof(uint32_t));
        nextAddress += sizeof(uint32_t);
        ;
    }

    m_eeprom->write(0, write_buff, imageSize);

    free(write_buff);
    return aditof::Status::OK;
}

Status ModuleMemory::writeModuleData(const std::string &ccbFileName, const std::string &cfgFileName) {

    if (!m_eeprom) {
        LOG(INFO) << "Write failed, cannot connect to module flash.";
        return Status::GENERIC_ERROR;
    }

    LOG(INFO) << "Generating Module Flash Image... ";
    LOG(INFO) << "   Loading CCB file: " << ccbFileName;
    LOG(INFO) << "   Loading CFG file: " << cfgFileName;

    uint8_t *ccbData = NULL;
    uint8_t *cfgData = NULL;
    uint32_t ccbSize = loadfileData(ccbFileName, &ccbData);
    uint32_t cfgSize = loadfileData(cfgFileName, &cfgData);

    Status status = createModuleImage(ccbData, ccbSize, cfgData, cfgSize);

    if (ccbData) {
        free(ccbData);
    }
    if (cfgData) {
        free(cfgData);
    }

    return status;
}

Status ModuleMemory::getSerialNumber(std::string &serialNumber)
{
    serialNumber = m_serialNumber;

    return aditof::Status::OK;
}

} // namespace aditof
