#include "offline_depth_sensor.h"
#include <cstring>
#include <fstream>
#include <iostream>

OfflineDepthSensor::OfflineDepthSensor(std::string path) : m_path(path) {
    m_sensorDetails.connectionType = aditof::ConnectionType::OFFLINE;
}

OfflineDepthSensor::~OfflineDepthSensor() = default;

aditof::Status OfflineDepthSensor::getFrame(uint16_t *buffer) {
    std::ifstream ifs(m_path.c_str());
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    for (int i = 0; i < content.size(); i += 2) {
        buffer[i / 2] = *(uint16_t *)&content[i];
    }
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::open() { return aditof::Status::OK; }

aditof::Status OfflineDepthSensor::start() { return aditof::Status::OK; }

aditof::Status OfflineDepthSensor::stop() { return aditof::Status::OK; }

aditof::Status OfflineDepthSensor::getAvailableFrameTypes(
    std::vector<aditof::DepthSensorFrameType> &types) {
    aditof::DepthSensorFrameType frameType;
    types = availableFrameTypes;
    return aditof::Status::OK;
}

aditof::Status
OfflineDepthSensor::setFrameType(const aditof::DepthSensorFrameType &type) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::program(const uint8_t *firmware,
                                           size_t size) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::readRegisters(const uint16_t *address,
                                                 uint16_t *data, size_t length,
                                                 bool burst /*=true*/) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::writeRegisters(const uint16_t *address,
                                                  const uint16_t *data,
                                                  size_t length,
                                                  bool burst /*=true*/) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::getAvailableControls(
    std::vector<std::string> &controls) const {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::setControl(const std::string &control,
                                              const std::string &value) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineDepthSensor::getDetails(aditof::SensorDetails &details) const {
    details = m_sensorDetails;
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::getHandle(void **handle) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineDepthSensor::getName(std::string &name) const {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_read_payload(uint8_t *payload,
                                                         uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status
OfflineDepthSensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                               uint16_t payload_len) {
    return aditof::Status::OK;
}

aditof::Status
OfflineDepthSensor::adsd3500_write_payload(uint8_t *payload,
                                           uint16_t payload_len) {
    return aditof::Status::OK;
}
