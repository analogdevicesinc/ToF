#include "offline_sensor_enumerator.h"
#include "offline_depth_sensor.h"

OfflineSensorEnumerator::OfflineSensorEnumerator() {
 //   m_sensorsInfo.emplace_back(std::string(std::string(RESOURCES) + "/offline/mode5_raw.bin"));
    m_sensorsInfo.emplace_back(std::string(std::string(RESOURCES) + "/offline/mode3_raw.bin"));
}

aditof::Status OfflineSensorEnumerator::getDepthSensors(std::vector<std::shared_ptr<aditof::DepthSensorInterface> > &depthSensors) {
    depthSensors.clear();

    depthSensors.emplace_back(std::make_shared<OfflineDepthSensor>(m_sensorsInfo.front()));
    return aditof::Status::OK;
}

aditof::Status OfflineSensorEnumerator::searchSensors() {
    return aditof::Status::OK;
}

aditof::Status OfflineSensorEnumerator::getStorages(
        std::vector<std::shared_ptr<aditof::StorageInterface>> &storages) {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status OfflineSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
        &temperatureSensors) {
    return aditof::Status::UNAVAILABLE;
}
