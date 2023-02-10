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
#ifndef DEPTH_SENSOR_INTERFACE_H
#define DEPTH_SENSOR_INTERFACE_H

#include <aditof/frame_definitions.h>
#include <aditof/sensor_definitions.h>
#include <aditof/status_definitions.h>

#include <cstddef>
#include <string>
#include <vector>

namespace aditof {

/**
 * @class DepthSensorInterface
 * @brief Provides access to the low level functionality of the camera sensor. This
 * includes sensor configuration as well as analog front end(AFE) configuration.
 */
class DepthSensorInterface {
  public:
    /**
     * @brief Destructor
     */
    virtual ~DepthSensorInterface() = default;

    /**
     * @brief Open the communication channels with the hardware.
     * @return Status
     */
    virtual aditof::Status open() = 0;

    /**
     * @brief Start the streaming of data from the sensor.
     * @return Status
     */
    virtual aditof::Status start() = 0;

    /**
     * @brief Stop the sensor data stream.
     * @return Status
     */
    virtual aditof::Status stop() = 0;

    /**
     * @brief Return all frame types that are supported by the sensor.
     * @param[out] types
     * @return Status
     */
    virtual aditof::Status getAvailableFrameTypes(
        std::vector<aditof::DepthSensorFrameType> &types) = 0;

    /**
     * @brief Set the sensor frame type to the given type
     * @param type - frame details structure containing the frame type
     * @return Status
     */
    virtual aditof::Status
    setFrameType(const aditof::DepthSensorFrameType &type) = 0;

    /**
     * @brief Program the sensor with the given firmware
     * @param firmware - chunk of data representin the firmware
     * @param size - the size of the firmware data in bytes
     * @return Status
     */
    virtual aditof::Status program(const uint8_t *firmware, size_t size) = 0;

    /**
     * @brief Request a frame from the sensor
     * @param buffer - a valid location where the new frame should be stored.
     * The size of the frame is known (cached) internally and gets updated each
     * time setFrameType() is called.
     * @return Status
     */
    virtual aditof::Status getFrame(uint16_t *buffer) = 0;

    /**
     * @brief Read multiple registers from AFE.
     * @param address - memory location pointing to addresses of registers to be
     * read
     * @param data - a valid location to store the content read from AFE
     * registers
     * @param length - the number of registers to read
     * @param burst - if enabled the function will read data starting from the provided
     * address; if disabled it will read data from the provided address list .
     * @return Status
     */
    virtual aditof::Status readRegisters(const uint16_t *address,
                                         uint16_t *data, size_t length,
                                         bool burst = true) = 0;

    /**
     * @brief Write to multiple AFE registers.
     * @param address - memory location pointing to addresses of registers to be
     * written
     * @param data - the location of the content to be written to AFE registers
     * @param length - the number of registers to write
     * @param burst - if enabled the function will write data starting from the provided
     * address; if disabled it will write data at the provided address list .
     * @return Status
     */
    virtual aditof::Status writeRegisters(const uint16_t *address,
                                          const uint16_t *data, size_t length,
                                          bool burst = true) = 0;

    /**
     * @brief Send a read command to adsd3500.
     * @param cmd - the command to be sent
     * @param[out] data - the variable where the read data will be stored
     * @param usDelay - the number of microseconds to wait between the host command
     * and the actual read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                             unsigned int usDelay = 0) = 0;

    /**
     * @brief Send a write command to adsd3500.
     * @param cmd - the command to be sent
     * @param data - the data to be written
     * @return Status
     */
    virtual aditof::Status adsd3500_write_cmd(uint16_t cmd, uint16_t data) = 0;

    /**
     * @brief Send a read command to adsd3500. This will perform a burst read making it
     *        useful for reading chunks of data.
     * @param cmd - the command to be sent
     * @param[out] readback_data - the location where the read data chunk will be stored
     * @param payload_len - the number of bytes to read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_payload_cmd(uint32_t cmd,
                                                     uint8_t *readback_data,
                                                     uint16_t payload_len) = 0;

    /**
     * @brief Reads a chunk of data from adsd3500. This will perform a burst read making it
     *        useful for reading chunks of data.
     * @param payload - the location from where to take the data chunk and read it
     * @param payload_len - the number of bytes to read
     * @return Status
     */
    virtual aditof::Status adsd3500_read_payload(uint8_t *payload,
                                                 uint16_t payload_len) = 0;

    /**
     * @brief Send a write command to adsd3500. This will perform a burst write making it
     *        useful for writing chunks of data.
     * @param cmd - the command to be sent
     * @param payload - the location from where to take the data chunk and write it
     * @param payload_len - the number of bytes to write
     * @return Status
     */
    virtual aditof::Status adsd3500_write_payload_cmd(uint32_t cmd,
                                                      uint8_t *payload,
                                                      uint16_t payload_len) = 0;

    /**
     * @brief Send a chunk of data (payload) to adsd3500. This will perform a burst write making it
     *        useful for writing chunks of data.
     * @param payload - the location from where to take the data chunk and write it
     * @param payload_len - the number of bytes to write
     * @return Status
     */
    virtual aditof::Status adsd3500_write_payload(uint8_t *payload,
                                                  uint16_t payload_len) = 0;

    /**
     * @brief Gets the sensors's list of controls
     * @param[out] controls
     * @return Status
     */
    virtual Status
    getAvailableControls(std::vector<std::string> &controls) const = 0;

    /**
     * @brief Sets a specific sensor control
     * @param[in] control - Control name
     * @param[in] value - Control value
     * @return Status
     */
    virtual Status setControl(const std::string &control,
                              const std::string &value) = 0;

    /**
     * @brief Gets the value of a specific sensor control
     * @param[in] control - Control name
     * @param[out] value - Control value
     * @return Status
     */
    virtual Status getControl(const std::string &control,
                              std::string &value) const = 0;

    /**
     * @brief Get a structure that contains information about the instance of
     * the sensor
     * @param[out] details - the variable where the sensor details should be
     * stored
     * @return Status
     */
    virtual aditof::Status getDetails(aditof::SensorDetails &details) const = 0;

    /**
     * @brief Gets a handle to be used by other devices such as Storage,
     * Temperature, etc. This handle will allow the other devices to
     * communicate remotely with the embedded target.
     * @param[out] handle - the handle which is owned by this instance
     * @return Status
     */
    virtual aditof::Status getHandle(void **handle) = 0;

    /**
     * @brief Get the name of the sensor
     * @param[out] name - the string in which the name is stored
     * @return Status
     */
    virtual aditof::Status getName(std::string &name) const = 0;
};

} // namespace aditof

#endif // DEPTH_SENSOR_INTERFACE_H
