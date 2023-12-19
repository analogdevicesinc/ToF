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
#ifndef FRAME_HANDLER_H
#define FRAME_HANDLER_H

#include <aditof/frame.h>
#include <aditof/status_definitions.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#define METADATA_SIZE 128

class FrameHandlerImpl;

class FrameHandler {
  public:
    SDK_API FrameHandler();
    SDK_API ~FrameHandler();

    /**
     * @brief Move constructor
     */
    SDK_API FrameHandler(FrameHandler &&) noexcept;
    /**
     * @brief Move assignment
     */
    SDK_API FrameHandler &operator=(FrameHandler &&) noexcept;

    /**
     * @brief Set the output path/output folder for the frames
     * @param[in] filePath - File path
     * @return Status
     */
    SDK_API aditof::Status setOutputFilePath(std::string &filePath);

    /**
     * @brief Set the input file for the frames
     * @param[in] fullFileName - Input file name
     * @return Status
     */
    SDK_API aditof::Status setInputFileName(std::string &fullFileName);

    /**
     * @brief Store frame to file
     * @param[in] frame - frame object to be stored in file
     * @param[in] fileName - the name of the file. If none provided
     * it will automatically generate one depending on time and frame number.
     * @return Status
     */
    SDK_API aditof::Status saveFrameToFile(aditof::Frame &frame,
                                           std::string fileName = "");

    /**
     * @brief Store frame to file using a separate thread.
     * @param[in] frame - frame object to be stored in file
     * @param[in] fileName - the name of the file. If none provided
     * it will automatically generate one depending on time and frame number.
     * @return Status
     */
    SDK_API aditof::Status
    saveFrameToFileMultithread(aditof::Frame *frame, std::string fileName = "");

    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    SDK_API aditof::Status readNextFrame(aditof::Frame &frame,
                                         std::string fullFileName = "");

    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    SDK_API aditof::Status setCustomFormat(std::string format);

    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    SDK_API aditof::Status storeFramesToSingleFile(bool enable);
    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    SDK_API aditof::Status setFrameContent(std::string frameContent);

  private:
    std::unique_ptr<FrameHandlerImpl> m_impl;
};

#endif // FRAME_HANDLER_H