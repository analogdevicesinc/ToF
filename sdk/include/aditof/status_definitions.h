/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
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
#ifndef STATUS_DEFINITIONS_H
#define STATUS_DEFINITIONS_H

#include "sdk_exports.h"

#include <ostream>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum Status
 * @brief Status of any operation that the TOF sdk performs.
 */
enum class Status {
    OK,               //!< Success
    BUSY,             //!< Device or resource is busy
    UNREACHABLE,      //!< Device or resource is unreachable
    INVALID_ARGUMENT, //!< Invalid arguments provided
    UNAVAILABLE,      //!< The requested action or resource is unavailable
    GENERIC_ERROR     //!< An error occured but there are no details available.
};

/**
 * @enum Adsd3500Status
 * @brief Status of the ADSD3500 sensor.
 */
enum class Adsd3500Status {
    OK,                       //!< Success
    INVALID_MODE,             //!< Invalid mode has been set
    INVALID_JBLF_FILTER_SIZE, //!< Invalid JBLF filter size has been set
    UNSUPPORTED_COMMAND,      //!< The command is not supported by the ADSD3500
    INVALID_MEMORY_REGION,    //!< Invalid memory region
    INVALID_FIRMWARE_CRC,     //!< Invalid firmware CRC
    INVALID_IMAGER,           //!< Invalid imager
    INVALID_CCB,              //!< Invalid CCB
    FLASH_HEADER_PARSE_ERROR, //!< Flash header parse error
    FLASH_FILE_PARSE_ERROR,   //!< Flash file parse error
    SPIM_ERROR,               //!< SPIM error
    INVALID_CHIPID,           //!< Invalid chip ID
    IMAGER_COMMUNICATION_ERROR, //!< Imager communication error
    IMAGER_BOOT_FAILURE,        //!< Imager boot failure
    FIRMWARE_UPDATE_COMPLETE,   //!< The firmware update action has completed
    NVM_WRITE_COMPLETE,         //!< The write action to the NVM has completed
    IMAGER_ERROR,               //!< Imager error
    UNKNOWN_ERROR_ID            //!< Unknown ID read from ADSD3500
};

/**
 * @brief operator << which make possible to print out items from aditof::Status enum
 *
 * @param os - output streamm
 * @param status - an item of type aditof::Status
 * @return std::ostream&
 */
SDK_API std::ostream &operator<<(std::ostream &os, aditof::Status status);

/**
 * @brief operator << which make possible to print out items from aditof::Adsd3500Status enum
 *
 * @param os - output streamm
 * @param status - an item of type aditof::Adsd3500Status
 * @return std::ostream&
 */
SDK_API std::ostream &operator<<(std::ostream &os,
                                 aditof::Adsd3500Status status);

} // namespace aditof

#endif // STATUS_DEFINITIONS_H
