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
#include "connections/target/target_sensor_enumerator.h"
#include "connections/target/adsd3100_sensor.h"
#include "connections/target/eeprom.h"
#include "connections/target/adsd3500_sensor.h"

using namespace aditof;

Status TargetSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        switch (sInfo.sensorType) {
        case SensorType::SENSOR_ADSD3500: {
            auto sensor = std::make_shared<Adsd3500Sensor>(
                sInfo.driverPath, sInfo.subDevPath, sInfo.captureDev);
            depthSensors.emplace_back(sensor);
            break;
        }
        case SensorType::SENSOR_ADSD3100: {
            auto sensor = std::make_shared<Adsd3100Sensor>(
                sInfo.driverPath, sInfo.subDevPath, sInfo.captureDev);
            depthSensors.emplace_back(sensor);
            break;
        }
        }
    }

    return Status::OK;
}

Status TargetSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {

    storages.clear();

    for (const auto &eInfo : m_storagesInfo) {
        auto storage =
            std::make_shared<Eeprom>(eInfo.driverName, eInfo.driverPath);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status TargetSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    for (const auto &tInfo : m_temperatureSensorsInfo) {
        switch (tInfo.sensorType) {
        case TempSensorType::NO_TYPE: {
            ; // do nothing
        }
        }
    }

    return Status::OK;
}
