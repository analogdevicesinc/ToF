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
#include <aditof/status_definitions.h>

namespace aditof {

std::ostream &operator<<(std::ostream &os, aditof::Status status) {
    switch (status) {
    case aditof::Status::OK:
        os << "Status::OK";
        break;
    case aditof::Status::BUSY:
        os << "Status::BUSY";
        break;
    case aditof::Status::UNREACHABLE:
        os << "Status::UNREACHABLE";
        break;
    case aditof::Status::INVALID_ARGUMENT:
        os << "Status::INVALID_ARGUMENT";
        break;
    case aditof::Status::UNAVAILABLE:
        os << "Status::UNAVAILABLE";
        break;
    case aditof::Status::GENERIC_ERROR:
        os << "Status::GENERIC_ERROR";
        break;
    default:
        os.setstate(std::ios_base::failbit);
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, aditof::Adsd3500Status status) {
    switch (status) {
    case aditof::Adsd3500Status::OK:
        os << "Adsd3500Status::OK";
        break;
    case aditof::Adsd3500Status::INVALID_MODE:
        os << "Adsd3500Status::INVALID_MODE";
        break;
    case aditof::Adsd3500Status::INVALID_JBLF_FILTER_SIZE:
        os << "Adsd3500Status::INVALID_JBLF_FILTER_SIZE";
        break;
    case aditof::Adsd3500Status::UNSUPPORTED_COMMAND:
        os << "Adsd3500Status::UNSUPPORTED_COMMAND";
        break;
    case aditof::Adsd3500Status::INVALID_MEMORY_REGION:
        os << "Adsd3500Status::INVALID_MEMORY_REGION";
        break;
    case aditof::Adsd3500Status::INVALID_FIRMWARE_CRC:
        os << "Adsd3500Status::INVALID_FIRMWARE_CRC";
        break;
    case aditof::Adsd3500Status::INVALID_IMAGER:
        os << "Adsd3500Status::INVALID_IMAGER";
        break;
    case aditof::Adsd3500Status::INVALID_CCB:
        os << "Adsd3500Status::INVALID_CCB";
        break;
    case aditof::Adsd3500Status::FLASH_HEADER_PARSE_ERROR:
        os << "Adsd3500Status::FLASH_HEADER_PARSE_ERROR";
        break;
    case aditof::Adsd3500Status::FLASH_FILE_PARSE_ERROR:
        os << "Adsd3500Status::FLASH_FILE_PARSE_ERROR";
        break;
    case aditof::Adsd3500Status::SPIM_ERROR:
        os << "Adsd3500Status::SPIM_ERROR";
        break;
    case aditof::Adsd3500Status::INVALID_CHIPID:
        os << "Adsd3500Status::INVALID_CHIPID";
        break;
    case aditof::Adsd3500Status::IMAGER_COMMUNICATION_ERROR:
        os << "Adsd3500Status::IMAGER_COMMUNICATION_ERROR";
        break;
    case aditof::Adsd3500Status::IMAGER_BOOT_FAILURE:
        os << "Adsd3500Status::IMAGER_BOOT_FAILURE";
        break;
    case aditof::Adsd3500Status::FIRMWARE_UPDATE_COMPLETE:
        os << "Adsd3500Status::FIRMWARE_UPDATE_COMPLETE";
        break;
    case aditof::Adsd3500Status::NVM_WRITE_COMPLETE:
        os << "Adsd3500Status::NVM_WRITE_COMPLETE";
        break;
    case aditof::Adsd3500Status::IMAGER_ERROR:
        os << "Adsd3500Status::IMAGER_ERROR";
        break;
    case aditof::Adsd3500Status::UNKNOWN_ERROR_ID:
        os << "Adsd3500Status::UNKNOWN_ERROR_ID";
        break;
    default:
        os.setstate(std::ios_base::failbit);
    }
    return os;
}

} // namespace aditof
