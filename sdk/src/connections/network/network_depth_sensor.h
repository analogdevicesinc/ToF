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
#ifndef NETWORK_DEPTH_SENSOR_H
#define NETWORK_DEPTH_SENSOR_H

#include "aditof/depth_sensor_interface.h"

#include <memory>

class NetworkDepthSensor : public aditof::DepthSensorInterface {
  public:
    NetworkDepthSensor(const std::string &name, const std::string &ip);
    ~NetworkDepthSensor();

  public: // implements DepthSensorInterface
    virtual aditof::Status open() override;
    virtual aditof::Status start() override;
    virtual aditof::Status stop() override;
    virtual aditof::Status getAvailableFrameTypes(
        std::vector<aditof::DepthSensorFrameType> &types) override;
    virtual aditof::Status
    setFrameType(const aditof::DepthSensorFrameType &type) override;
    virtual aditof::Status program(const uint8_t *firmware,
                                   size_t size) override;
    virtual aditof::Status getFrame(uint16_t *buffer) override;
    virtual aditof::Status readRegisters(const uint16_t *address,
                                            uint16_t *data,
                                            size_t length, bool burst = true) override;
    virtual aditof::Status writeRegisters(const uint16_t *address,
                                             const uint16_t *data,
                                             size_t length, bool burst = true) override;
    virtual aditof::Status
    getDetails(aditof::SensorDetails &details) const override;
    virtual aditof::Status getHandle(void **handle) override;
    virtual aditof::Status getName(std::string &name) const override;

    virtual aditof::Status pulsatrix_read_cmd(uint16_t cmd, uint16_t *data) override;
    virtual aditof::Status pulsatrix_write_cmd(uint16_t cmd, uint16_t data) override;
    virtual aditof::Status pulsatrix_read_payload_cmd(uint32_t cmd, uint8_t* readback_data, uint16_t payload_len) override;
    virtual aditof::Status pulsatrix_write_payload_cmd(uint32_t cmd, uint8_t* payload, uint16_t payload_len) override;
    virtual aditof::Status pulsatrix_write_payload(uint8_t* payload, uint16_t payload_len) override;
  
  private:
    struct ImplData;
    std::string m_sensorName;
    aditof::SensorDetails m_sensorDetails;
    std::unique_ptr<ImplData> m_implData;
    int m_sensorIndex;
    static int m_sensorCounter;
};

#endif // NETWORK_DEPTH_SENSOR_H
