#include "offline_sensor_enumerator.h"
#include "offline_depth_sensor.h"

OfflineSensorEnumerator::OfflineSensorEnumerator() {
    m_sensorsInfo.emplace_back(
        std::string(std::string(RESOURCES) + "/offline"));
}

aditof::Status OfflineSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &depthSensors) {
    depthSensors.clear();

    depthSensors.emplace_back(
        std::make_shared<OfflineDepthSensor>(m_sensorsInfo.front()));
    return aditof::Status::OK;
}

aditof::Status OfflineSensorEnumerator::searchSensors() {
    return aditof::Status::OK;
}

aditof::Status
OfflineSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    return aditof::Status::UNAVAILABLE;
}
