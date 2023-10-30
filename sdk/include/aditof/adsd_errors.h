/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ADSD3500_DEFS
#define ADSD3500_DEFS

#include <cstdint>
#include <string>
#include <unordered_map>

namespace aditof {

/**
 * @class ADSDErrors
 * @brief Error codes from the ADSD3500, ADSD3100, ADSD3030
 * Usage:
 * Included with aditof/camera.h
 * For example: 
 *    ADSDErrors err;
 *    LOG(INFO) << ADSDErrors::ADSD3500_STATUS_INVALID_MODE;
 *    LOG(INFO) << err.GetString(ADSDErrors::ADSD3500, 0x0001);
 */
class ADSDErrors {
  public:
    /**
     * @brief The hardware types for the error for GetString.
     */
    enum ADSD3xxx { ADSD3500, ADSD3100, ADSD3030 };

    ADSDErrors() {
        m_ADSD3100ErrLookup = {
            {ADSD3100_ERR_MODE_USECASE, "Invalid mode selection."},
            {ADSD3100_ERR_MODE_MODE_DRIVER, "Invalid LD mode selection."},
            {ADSD3100_ERR_PLLLOCK_LOCK1, "PLLLOCK error location 1."},
            {ADSD3100_ERR_PLLLOCK_LOCK2, "PLLLOCK error location 2."},
            {ADSD3100_ERR_PLLLOCK_LOCK3, "PLLLOCK error location 3."},
            {ADSD3100_ERR_OVERHEAT_IMAGER, "Imager sensor overheat."},
            {ADSD3100_ERR_OVERHEAT_LD, "Laser driver overheat."},
            {ADSD3100_ERR_LASER_CHIPID, "Laser driver invalid chip ID."},
            {ADSD3100_ERR_LASER_LPS, "Corrupted laser driver data."},
            {ADSD3100_ERR_LASER_NO_DIFFUSER, "Laser diffuser problem."},
            {ADSD3100_ERR_LASER_SHORT, "Laser driver shorted to GND."},
            {ADSD3100_ERR_LVDS_HIGH_DC, "Laser driver duty cycle too large."},
            {ADSD3100_ERR_LVDS_PULSE_LONG,
             "Laser driver active time too long."},
            {ADSD3100_ERR_LVDS_OPEN_SHORT,
             "Laser driver input open or short detected."},
            {ADSD3100_ERR_LASER_LONG_LEN_ON,
             "Laser driver enabled for too long of time."},
            {ADSD3100_ERR_LASER_SHORT_LEN_OFF,
             "Laser driver disabled for too short of time."},
            {ADSD3100_ERR_LASER_LPS_READ, "Laser driver corrupted data."},
            {ADSD3100_ERR_LASER_VLD_LOW, "Laser driver supply too low."},
            {ADSD3100_ERR_LASER_VLD_HIGH, "Laser driver supply too high."}};

        m_ADSD3500StatusLookup = {
            {ADSD3500_STATUS_INVALID_MODE, "Mode selected is invalid."},
            {ADSD3500_STATUS_INVALID_JBLF_FILTER_SIZE,
             "The JBLF filter size speficied is incorrect."},
            {ADSD3500_STATUS_UNSUPPORTED_CMD,
             "An unsupported command was sent to the ASDD3500."},
            {ADSD3500_STATUS_INVALID_MEMORY_REGION,
             "A register write or read operation does not match any valid "
             "region."},
            {ADSD3500_STATUS_INVALID_FIRMWARE_CRC,
             "The ADSD3500 firmware CRC check failed."},
            {ADSD3500_STATUS_INVALID_IMAGER,
             "The imager firmware is not valid."},
            {ADSD3500_STATUS_INVALID_CCB, "The imager CCB file is not valid."},
            {ADSD3500_STATUS_FLASH_HEADER_PARSE_ERROR, "Flash update error."},
            {ADSD3500_STATUS_FLASH_FILE_PARSE_ERROR, "Flash update error."},
            {ADSD3500_STATUS_SPIM_ERROR,
             "SPI Master error occured, which this impacts the ADSD3500 - "
             "image communication."},
            {ADSD3500_STATUS_INVALID_CHIPID, "The image chip ID is invalid."},
            {ADSD3500_STATUS_IMAGER_COMMUNICATION_ERROR,
             "SPI Master error occured during communication between the "
             "ASDSD3500 and the imager."},
            {ADSD3500_STATUS_IMAGER_BOOT_FAILURE, "Unable to boot the imager."},
            {ADSD3500_STATUS_IMAGER_ERROR, "The imager reported an error."},
            {ADSD3500_STATUS_TIMEOUT_ERROR,
             "This is when timer is expired but ADSD3500 is not able to send "
             "out frame due to some error."},
            {ADSD3500_STATUS_DYNAMIC_MODE_SWITCHING_NOT_ENABLED,
             "Dynamic mode switching is being set, but it is not enabled."},
            {ADSD3500_STATUS_INVALID_DYNAMIC_MODE_COMPOSITIONS,
             "The selected dyanamic mode configuration is not valid."},
            {ADSD3500_STATUS_INVALID_PHASE_INVALID_VALUE,
             "An incorrect phase invalid value specified."},
            {ADSD3500_STATUS_FIRMWARE_UPDATE_COMPLETE,
             "Firmware update is complete."},
            {ADSD3500_STATUS_NVM_WRITE_COMPLETE, "NVM update is complete."}};
    }

    /**
     * @brief Returns a string for a given target (adsdType) and error code.
     * @param[in] adsdType - The source of the error: ADSD3500, ADSD3100 or ADSD3030.
     * @param[in] value - Error value
     * @return Error string
     */
    std::string GetString(ADSD3xxx adsdType, uint16_t value) {

        std::string ret = "";

        if (adsdType == ADSD3500) {
            auto it = m_ADSD3500StatusLookup.find(value);

            if (it != m_ADSD3500StatusLookup.end()) {
                ret = m_ADSD3500StatusLookup[value];
            }
        } else if (adsdType == ADSD3100) {
            auto it = m_ADSD3100ErrLookup.find(value);

            if (it != m_ADSD3100ErrLookup.end()) {
                ret = m_ADSD3100ErrLookup[value];
            }
        }

        return ret;
    }

  public:
    // ADSD3500 error codes read via the "Get Status" (0x0020) command
    /**
     * @brief Mode selected is invalid.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_MODE = 0x0001;
    /**
     * @brief The JBLF filter size speficied is incorrect.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_JBLF_FILTER_SIZE = 0x0002;
    /**
     * @brief An unsupported command was sent to the ASDD3500.
     */
    static const uint16_t ADSD3500_STATUS_UNSUPPORTED_CMD = 0x0003;
    /**
     * @brief TODO
     */
    static const uint16_t ADSD3500_STATUS_INVALID_MEMORY_REGION = 0x0004;
    /**
     * @brief The ADSD3500 firmware CRC check failed.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_FIRMWARE_CRC = 0x0005;
    /**
     * @brief The imager firmware is not valid.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_IMAGER = 0x0006;
    /**
     * @brief The imager CCB file is not valid.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_CCB = 0x0007;
    /**
     * @brief Flash update error.
     */
    static const uint16_t ADSD3500_STATUS_FLASH_HEADER_PARSE_ERROR = 0x0008;
    /**
     * @brief Flash update error.
     */
    static const uint16_t ADSD3500_STATUS_FLASH_FILE_PARSE_ERROR = 0x0009;
    /**
     * @brief SPI Master error occured, which this impacts the ADSD3500 - image communication.
     */
    static const uint16_t ADSD3500_STATUS_SPIM_ERROR = 0x000A;
    /**
     * @brief The imager chip ID is invalid.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_CHIPID = 0x000B;
    /**
     * @brief SPI Master error occured during communication between the ASDSD3500 and the imager.    
     */
    static const uint16_t ADSD3500_STATUS_IMAGER_COMMUNICATION_ERROR = 0x000C;
    /**
     * @brief Unable to boot the imager.
     */
    static const uint16_t ADSD3500_STATUS_IMAGER_BOOT_FAILURE = 0x000D;
    /**
     * @brief The imager reported an error.
     */
    static const uint16_t ADSD3500_STATUS_IMAGER_ERROR = 0x0010;
    /**
     * @brief This is when timer is expired but ADSD3500 is not able to send out frame due to some error.
     */
    static const uint16_t ADSD3500_STATUS_TIMEOUT_ERROR = 0x0011;
    /**
     * @brief Dynamic mode switching is being set, but it is not enabled.
     */
    static const uint16_t ADSD3500_STATUS_DYNAMIC_MODE_SWITCHING_NOT_ENABLED =
        0x0013;
    /**
     * @brief The selected dyanamic mode configuration is not valid.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_DYNAMIC_MODE_COMPOSITIONS =
        0x0014;
    /**
     * @brief An incorrect phase invalid value specified.
     */
    static const uint16_t ADSD3500_STATUS_INVALID_PHASE_INVALID_VALUE = 0x0015;

    // ADSD3500 status codes read via the "Get Status" (0x0020) command
    /**
     * @brief Firmware update is complete.
     */
    static const uint16_t ADSD3500_STATUS_FIRMWARE_UPDATE_COMPLETE = 0x000E;
    /**
     * @brief NVM update is complete.
     */
    static const uint16_t ADSD3500_STATUS_NVM_WRITE_COMPLETE = 0x000F;

    /**
     * @brief Invalid mode selection.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_MODE_USECASE = 0x0001;
    /**
     * @brief Invalid LD mode selection.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_MODE_MODE_DRIVER = 0x0002;
    /**
     * @brief PLLLOCK error location 1.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_PLLLOCK_LOCK1 = 0x0004;
    /**
     * @brief PLLLOCK error location 2.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_PLLLOCK_LOCK2 =
        0x0008; // PLLLOCK error location 2.
    /**
     * @brief PLLLOCK error location 3.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_PLLLOCK_LOCK3 =
        0x000C; // PLLLOCK error location 3.
    /**
     * @brief Imager sensor overheat.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_OVERHEAT_IMAGER = 0x0010;
    /**
     * @brief Laser driver overheat.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_OVERHEAT_LD = 0x0020;
    /**
     * @brief Laser driver invalid chip ID.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_CHIPID = 0x0040;
    /**
     * @brief Corrupted laser driver data.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_LPS = 0x0080;
    /**
     * @brief Laser diffuser problem.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_NO_DIFFUSER = 0x0100;
    /**
     * @brief Laser driver shorted to GND.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_SHORT = 0x0140;
    /**
     * @brief Laser driver duty cycle too large.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LVDS_HIGH_DC = 0x0180;
    /**
     * @brief Laser driver active time too long.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LVDS_PULSE_LONG = 0x01C0;
    /**
     * @brief Laser driver input open or short detected.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LVDS_OPEN_SHORT = 0x0200;
    /**
     * @brief Laser driver enabled for too long of time.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_LONG_LEN_ON = 0x0240;
    /**
     * @brief Laser driver disabled for too short of time.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_SHORT_LEN_OFF = 0x0280;
    /**
     * @brief Laser driver corrupted data.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_LPS_READ = 0x02C0;
    /**
     * @brief Laser driver supply too low.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_VLD_LOW = 0x0300;
    /**
     * @brief Laser driver supply too high.
     * Imager error codes read via "Get Imager Error Code" (0x0038) command
     * Note, this only valid if "Get Status" (0x0020) reports a value of "ADSD3500_STATUS_IMAGER_ERROR" (0x0010)
     */
    static const uint16_t ADSD3100_ERR_LASER_VLD_HIGH = 0x0340;

  private:
    std::unordered_map<uint16_t, std::string> m_ADSD3500StatusLookup;
    std::unordered_map<uint16_t, std::string> m_ADSD3100ErrLookup;
};

} // namespace aditof
#endif //ADSD3500_DEFS