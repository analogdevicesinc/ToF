#include "offline_depth_sensor.h"
#include "tofi/tofi_config.h"
#include <cstring>
#ifdef UNIX
#include <dirent.h>
#endif
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>

OfflineDepthSensor::OfflineDepthSensor(std::string path) : m_path(path) {
    m_sensorDetails.connectionType = aditof::ConnectionType::OFFLINE;
    m_connectionType = "offline";
}

OfflineDepthSensor::~OfflineDepthSensor() {
    for (auto index : m_frameTypes) {
        delete[] index.second.first;
    }
    m_frameTypes.clear();
}

aditof::Status OfflineDepthSensor::getFrame(uint16_t *buffer,
                                            uint32_t *bufferSize) {
    auto frame = m_frameTypes.find(m_frameTypeSelected + ".bin");
    if (frame == m_frameTypes.end()) {
        LOG(WARNING) << "get frame error " << m_frameTypeSelected
                     << " is not found";
        return aditof::Status::GENERIC_ERROR;
    }
    memcpy(buffer, frame->second.first, frame->second.second);

    if (bufferSize)
        *bufferSize = frame->second.second;

    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::open() {
    std::vector<std::string> frameTypesResources;
    std::streampos size;
    uint16_t *buffer;
#ifdef UNIX
    std::string fileName;
    struct dirent *entry;
    DIR *dir = opendir(m_path.c_str());
    while ((entry = readdir(dir)) != NULL) {
        fileName = entry->d_name;
        if (fileName[0] != ".")
            frameTypesResources.push_back(fileName);
    }
#elif WIN32
    HANDLE dir;
    WIN32_FIND_DATA file_data;
    if ((dir = FindFirstFile((m_path + "/*").c_str(), &file_data)) ==
        INVALID_HANDLE_VALUE)
        return aditof::Status::UNREACHABLE;
    do {
        std::string fileName = file_data.cFileName;
        const bool is_directory =
            (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

        if (fileName[0] == '.')
            continue;

        if (is_directory)
            continue;

        frameTypesResources.push_back(fileName);
    } while (FindNextFile(dir, &file_data));

    FindClose(dir);
#endif
    if (frameTypesResources.empty()) {
        LOG(WARNING) << "No available frame types found";
        return ::aditof::Status::INVALID_ARGUMENT;
    } else {
        for (int index = 0; index < frameTypesResources.size(); index++) {
            std::ifstream ifs;
            ifs.open(m_path + '/' + frameTypesResources[index],
                     std::ios::in | std::ios::binary);
            std::string content((std::istreambuf_iterator<char>(ifs)),
                                (std::istreambuf_iterator<char>()));
            buffer = new uint16_t[content.size() / 2];
            memcpy(buffer, content.data(), content.size());
            m_frameTypes.insert({frameTypesResources[index],
                                 std::make_pair(buffer, content.size())});
            ifs.close();
        }
    }
    return aditof::Status::OK;
}

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
    m_frameTypeSelected = type.type;
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
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::getControl(const std::string &control,
                                              std::string &value) const {
    if (control == "imagerType")
        value = "1";
    else if (control == "modeInfoVersion")
        value = "2";
    return aditof::Status::OK;
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
    name = m_connectionType;
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_read_cmd(uint16_t cmd,
                                                     uint16_t *data,
                                                     unsigned int usDelay) {
    return aditof::Status::OK;
}

aditof::Status
OfflineDepthSensor::setHostConnectionType(std::string &connectionType) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_write_cmd(uint16_t cmd,
                                                      uint16_t data) {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_read_payload_cmd(
    uint32_t cmd, uint8_t *readback_data, uint16_t payload_len) {
    TofiXYZDealiasData dealiasStruct;
    if (cmd == 0x01) {
        dealiasStruct.camera_intrinsics.fx = 783.19696;
        dealiasStruct.camera_intrinsics.fy = 783.248596;
        dealiasStruct.camera_intrinsics.cx = 514.108887;
        dealiasStruct.camera_intrinsics.cy = 515.469894;
        dealiasStruct.camera_intrinsics.codx = 0;
        dealiasStruct.camera_intrinsics.cody = 0;
        dealiasStruct.camera_intrinsics.k1 = -0.12491288;
        dealiasStruct.camera_intrinsics.k2 = 0.025611693;
        dealiasStruct.camera_intrinsics.k3 = 0.102783337;
        dealiasStruct.camera_intrinsics.k4 = 0.222312465;
        dealiasStruct.camera_intrinsics.k5 = -0.134152338;
        dealiasStruct.camera_intrinsics.k6 = 0.190698504;
        dealiasStruct.camera_intrinsics.p2 = 5.45154289e-05;
        dealiasStruct.camera_intrinsics.p1 = 0.000267527241;
        memcpy(readback_data, &dealiasStruct.camera_intrinsics,
               sizeof(CameraIntrinsics));
    } else if (cmd == 0x02) {
        if (readback_data[0] == 3) {
            dealiasStruct.n_rows = 512;
            dealiasStruct.n_cols = 512;
            dealiasStruct.row_bin_factor = 1;
            dealiasStruct.col_bin_factor = 1;
            dealiasStruct.n_sensor_rows = 1024;
            dealiasStruct.n_sensor_cols = 1024;
        } else if (readback_data[0] == 2) {
            dealiasStruct.n_rows = 512;
            dealiasStruct.n_cols = 512;
            dealiasStruct.row_bin_factor = 2;
            dealiasStruct.col_bin_factor = 2;
            dealiasStruct.n_sensor_rows = 1024;
            dealiasStruct.n_sensor_cols = 1024;
        }
        dealiasStruct.n_freqs = 2;
        dealiasStruct.n_offset_rows = 0;
        dealiasStruct.n_offset_cols = 0;
        dealiasStruct.FreqIndex[0] = 1;
        dealiasStruct.FreqIndex[1] = 2;
        dealiasStruct.FreqIndex[2] = 0;
        dealiasStruct.Freq[0] = 14200;
        dealiasStruct.Freq[1] = 17750;
        dealiasStruct.Freq[2] = 0;
        memcpy(readback_data, &dealiasStruct,
               sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));
    } else {
        memset(readback_data, 1, payload_len);
    }
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

aditof::Status OfflineDepthSensor::adsd3500_reset() {
    return aditof::Status::OK;
}

aditof::Status OfflineDepthSensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback cb) {
    LOG(WARNING)
        << "Registering an interrupt callback on an offline connection "
           "is not supported!";
    return aditof::Status::UNAVAILABLE;
}
