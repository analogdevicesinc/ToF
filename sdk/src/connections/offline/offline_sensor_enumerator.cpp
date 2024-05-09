#include "offline_sensor_enumerator.h"
#include "offline_depth_sensor.h"

OfflineSensorEnumerator::OfflineSensorEnumerator() {
#if TARGET
    std::string pathToFrames =
        std::string(RESOURCES) + std::string("/offline/adsd3500_raw");
#else
    std::string pathToFrames = std::string(RESOURCES) + std::string("/offline");
#endif
    m_sensorsInfo.emplace_back(pathToFrames);
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
