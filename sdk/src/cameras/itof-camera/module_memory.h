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
#ifndef TOF_MODULE_MEMORY_H
#define TOF_MODULE_MEMORY_H

#include <cstdint>
#include <memory>
#include "aditof/storage_interface.h"

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum Flash_Image_Version
 * @brief Defines flash image verison for writing
 */
enum Flash_Image_Version : uint8_t { 
    FLASH_MAJOR_VERSION = 1, ///< major version of flash format
    FLASH_MINOR_VERSION = 0  ///< minor version of flash format
};

/**
 * @enum TOF_ChunkID_t
 * @brief Identifies manufacturer ID
 */
typedef enum {
    CHUNK_ID_ANALOG_DEVICES = 0xAD, ///< Analog Devices MFG Identifier 
} TOF_ChunkID_t;

/**
 * @enum TOF_ChunkType_t
 * @brief Defines chunk ID types
 */
typedef enum {
    CHUNK_TYPE_CCB = 1, ///< chunk is CCB (calibration) format
    CHUNK_TYPE_CFG = 2, ///< chunk is CFG (configuration) format
    CHUNK_TYPE_CAP_STRUCTURE_HEADER = 0x50, ///< Flash Capabilities Structure Header
    CHUNK_TYPE_DEBUG_INFO_HEADER = 0x51, ///< Debug/Diagnostic logging area header
    CHUNK_TYPE_INIT_FIRMWARE_HEADER = 0x52, ///< Init firmware (boot rom extension)
    CHUNK_TYPE_ADSD3500_FIRMWARE_FACTORY_HEADER = 0x53, ///< Adsd3500 firmware factory default header
    CHUNK_TYPE_ADSD3500_FIRMWARE_CURRENT_HEADER = 0x54, ///< Adsd3500 firmware current header
    CHUNK_TYPE_ADSD3500_FIRMWARE_UPGRADE_HEADER = 0x55, ///< Adsd3500 upgrade firmware header
    CHUNK_TYPE_IMAGER_FIRMWARE_FACTORY_HEADER = 0x56, ///< Imager firmware factory default header
    CHUNK_TYPE_IMAGER_FIRMWARE_CURRENT_HEADER = 0x57, ///< Imager firmware current header
    CHUNK_TYPE_IMAGER_FIRMWARE_UPGRADE_HEADER = 0x58, ///< Imager upgrade firmware header
    CHUNK_TYPE_FILE_HEADER = 0x3D ///< file header chunk
} TOF_ChunkType_t;

/**
 * @struct TOF_ChunkVesion_t
 * @brief Describes chunk type and version
 */
typedef struct {
        uint8_t chunkid; ///< TOF_ChunkID_t value            
        uint8_t chunktype; ///< TOF_ChunkType_t value
        uint8_t major; ///< major version of chunk
        uint8_t minor; ///< minor version of chunk
} TOF_ChunkVesion_t;

/**
 * @struct TOF_ChunkHeader_t
 * @brief Header for each memory chunk
 */
typedef struct tag_chunkHeader {
    TOF_ChunkVesion_t revision;         ///< chunk id type and version
    uint32_t          headerSizeBytes;  ///< chunk header size, i.e. sizeof(TOF_ChunkHeader_t)                                    
    uint32_t          chunkSizeBytes;   ///< chunk data size + 4bytes CRC, does not include chunk header
    uint32_t          nextChunkAddress; ///< memory start address of next chunk, chucks can be non-continguous
    uint32_t          chunkHeaderCRC;  ///< crc of the chunk header
} TOF_ChunkHeader_t;

typedef struct moduleFiles {
    std::string jsonFile;
    std::string ccbFile;
    std::string cfgFile;
} TOF_ModuleFiles_t;

/**
 * @class ModuleMemory
 * @brief Helper class to read/write, generate and parse a camera module eeprom/flash image 
 */
class ModuleMemory {
public:
    /**
     * @brief Constructor
     * @param[in] - module memory device specific implementataion (eeprom/flash)
     * @see StorageInterface
     */
    ModuleMemory(std::shared_ptr<StorageInterface> eeprom) :m_eeprom(eeprom) {}
    
    /**
     * @brief Destructor
     */
    ~ModuleMemory() = default;

    /**
     * @brief Read calibration and configuration data from camera module memory, writes to temporary file(s).
     * A valid camera memory will contain a CCB file and optionally a firmware (CFG) file. A temporary 
     * configuration (JSON) file is generated, appropriate for calling aditof::Camera::initialize(). 
     * The temporary files must be manually deleted before the application exits. 
     * @param[in] device - device interface connected to module memory flash/eeprom
     * @param[out] tempJsonFile - temporary filename for configuration json file.
     * @param[out] tempFiles - all temporary file generated, these files should be deleted by the caller on close.
     * @return aditof::Status no temporary files are created or returned on an error
     * @see aditof::Status
     */
    Status readModuleData(std::string &tempJsonFile, TOF_ModuleFiles_t &tempFiles );

    /**
     * @brief Write calibration (CCB file) and optionally configuration data (CFG file) to camera module Flash.
     * This will erase and overwrite attached camera module flash.
     * @param[in] device - device interface connected to module memory flash/eeprom.
     * @param[in] ccbFileName - filename for calibration ccb file.
     * @param[in] cfgFileName - optional filename for configuration cfg file, pass an empty string if not defined.
     * @return aditof::Status
     * @see aditof::Status
     */    
    Status writeModuleData(const std::string &ccbFileName, const std::string &cfgFileName );

    /**
     * @brief Read calibration data from camera module memory, writes to temporary file.
     * For legacy support of camera modules that only contain raw CCB data in flash (deprecated). 
     * A configuration (JSON) file is generated, appropriate for calling aditof::Camera::initialize(). 
     * The temporary files must be manually deleted before the application exits. 
     * @param[in] device - device interface connected to module memory flash/eeprom
     * @param[out] tempJsonFile - temporary filename for configuration json file.
     * @param[out] tempFiles - all temporary file generated, these files should be deleted by the caller on close.
     * @param[out] ccbSerialNumber - serial number from CCB header
     * @return aditof::Status no temporary files are created or returned on an error
     * @see aditof::Status
     */
    Status readLegacyModuleCCB(std::string &tempJsonFile, TOF_ModuleFiles_t &tempFiles, std::string &ccbSerialNumber);

    /**
     * @brief Returns the serial number stored in the camera module memory.
     * @param[out] ccbSerialNumber - serial number from CCB header
     * @return aditof::Status whether the operation succeeded or an error has occurred
     * @see aditof::Status
     */
    Status getSerialNumber(std::string &serialNumber);

    /**
     * @brief Process NVM firmware based on header chunk type.
     * @param[in] chunkType - NVM header chunk type
     * @param[in] pChunkData - NVM chunk data to be written in temp file
     * @param[in] payloadSize - Data size - CRC in bytes
     * @return aditof::Status whether the operation succeeded or an error has occurred
     * @see aditof::Status
     */
    Status processNVMFirmware(uint8_t chunkType, uint8_t *pChunkData, int payloadSize);

     /**
     * @brief Displays CRC error according to chunk type
     * @param[in] chunkType - NVM header chunk type
     */
    void displayCRCError(uint8_t chunkType);

private:
    std::string ccbFilename;
    std::string cfgFilename;
    std::string jsonFilename;
    bool _badCFG = false;
    bool _badCCB = false;
    bool isBadChunk = false;

  protected:
    bool verifyChunkHeader(const TOF_ChunkHeader_t *const pChunkHeader);
    Status readChunkHeader(uint8_t *const buffer, int32_t chunkDataAddr );
    Status writeData(uint32_t address, const uint8_t *const data, uint32_t length);
    uint32_t loadfileData(const std::string filename, uint8_t **dataBufferOut);
    TOF_ChunkHeader_t initChunckHeader(TOF_ChunkType_t type, uint32_t address, uint32_t dataSize, bool isLastChunk );
    Status createModuleImage(const uint8_t *ccbData, uint32_t ccbSize, const uint8_t *cfgData, uint32_t cfgSize);

    std::string writeToTempFile(const uint8_t *const dataBuffer, const uint32_t length, std::string fileTag);
    std::string writeTempCCB(const uint8_t *const ccbFileData, const uint32_t length);
    std::string writeTempCFG(const uint8_t *const cfgFileData, const uint32_t length);
    std::string writeTempJSON(const std::string ccbFilename, const std::string cfgFilename);

    std::shared_ptr<StorageInterface> m_eeprom;
    std::string m_serialNumber;
};

} // namespace aditof

#endif //TOF_MODULE_MEMORY_H