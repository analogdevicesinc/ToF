#include "tof-sdk-interface.h"
#include <glog/logging.h>
#include "buffer.pb.h"

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>
#include <aditof/sensor_enumerator_factory.h>
#include <aditof/sensor_enumerator_interface.h>
#include <aditof/storage_interface.h>
#include <aditof/temperature_sensor_interface.h>

/* Available sensors */
std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
std::vector<std::shared_ptr<aditof::StorageInterface>> storages;
std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
    temperatureSensors;
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;

int init_tof_sdk(char* cap_dev_path) {
    auto sensorsEnumerator =
        aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorsEnumerator) {
        DLOG(ERROR) << "Failed to construct a sensors enumerator!\n";
        return 1;
    }

    sensorsEnumerator->searchSensors();
    sensorsEnumerator->getDepthSensors(depthSensors);
    sensorsEnumerator->getStorages(storages);
    sensorsEnumerator->getTemperatureSensors(temperatureSensors);

    if (depthSensors.size() < 1) {
        DLOG(ERROR) << "No camera sensor are available!\n";
        return 1;
    }

    camDepthSensor = depthSensors[0];

    return 0;
}

/* ---------------------------------------------------------------------------
 * Time of flight SDK related
 */

void convertDepthSensorFrameTypesToProtoMsg(
    std::vector<aditof::DepthSensorFrameType> depthSensorFrameTypes,
    uvc_payload::DepthSensorFrameTypeVector &depthSensorFrameTypesPayload) {
    for (const aditof::DepthSensorFrameType &depthSensorFrameType :
         depthSensorFrameTypes) {
        LOG(INFO) << depthSensorFrameType.type << " "
                  << depthSensorFrameType.width << " "
                  << depthSensorFrameType.content.size();
        uvc_payload::DepthSensorFrameType *depthSensorFrameTypePayload =
            depthSensorFrameTypesPayload.add_depthsensorframetypes();
        depthSensorFrameTypePayload->set_type(depthSensorFrameType.type);
        depthSensorFrameTypePayload->set_width(depthSensorFrameType.width);
        depthSensorFrameTypePayload->set_height(depthSensorFrameType.height);

        for (const aditof::DepthSensorFrameContent &depthSensorFrameContent :
             depthSensorFrameType.content) {
            uvc_payload::DepthSensorFrameContent
                *depthSensorFrameContentPayload =
                    depthSensorFrameTypePayload->add_depthsensorframecontent();
            depthSensorFrameContentPayload->set_type(
                depthSensorFrameContent.type);
            depthSensorFrameContentPayload->set_width(
                depthSensorFrameContent.width);
            depthSensorFrameContentPayload->set_height(
                depthSensorFrameContent.height);
        }
    }
}

void convertProtoMsgToDepthSensorFrameType(
    const uvc_payload::DepthSensorFrameType &protoMsg,
    aditof::DepthSensorFrameType &aditofStruct) {
    aditofStruct.type = protoMsg.type();
    aditofStruct.width = protoMsg.width();
    aditofStruct.height = protoMsg.height();
    for (int i = 0; i < protoMsg.depthsensorframecontent_size(); ++i) {
        aditof::DepthSensorFrameContent content;

        content.type = protoMsg.depthsensorframecontent(i).type();
        content.width = protoMsg.depthsensorframecontent(i).width();
        content.height = protoMsg.depthsensorframecontent(i).height();
        aditofStruct.content.emplace_back(content);
    }
}

const char* handleClientRequest(const char *buf) {
    uvc_payload::ClientRequest request;
    uvc_payload::ServerResponse response;
    std::string serverResponseBlob;
    char *string_response;

    request.ParseFromString(buf);

    switch (request.func_name()) {

    case uvc_payload::FunctionName::SEARCH_SENSORS: {
        // Depth sensor
        if (depthSensors.size() < 1) {
            response.set_message("No depth sensors are available");
            response.set_status(::uvc_payload::Status::UNREACHABLE);
            break;
        }

        aditof::SensorDetails depthSensorDetails;
        camDepthSensor->getDetails(depthSensorDetails);
        auto pbSensorsInfo = response.mutable_sensors_info();

        // Storages
        int storage_id = 0;
        for (const auto &storage : storages) {
            std::string name;
            storage->getName(name);
            auto pbStorageInfo = pbSensorsInfo->add_storages();
            pbStorageInfo->set_name(name);
            pbStorageInfo->set_id(storage_id);
            ++storage_id;
        }

        // Temperature sensors
        int temp_sensor_id = 0;
        for (const auto &sensor : temperatureSensors) {
            std::string name;
            sensor->getName(name);
            auto pbTempSensorInfo = pbSensorsInfo->add_temp_sensors();
            pbTempSensorInfo->set_name(name);
            pbTempSensorInfo->set_id(temp_sensor_id);
            ++temp_sensor_id;
        }

        response.set_status(
            static_cast<::uvc_payload::Status>(aditof::Status::OK));

        break;
    }

    case uvc_payload::FunctionName::GET_AVAILABLE_FRAME_TYPES: {
        std::vector<aditof::DepthSensorFrameType> frameTypes;
        auto depthSensorFrameTypesMsg =
            response.mutable_available_frame_types();

        camDepthSensor->getAvailableFrameTypes(frameTypes);
        convertDepthSensorFrameTypesToProtoMsg(frameTypes,
                                               *depthSensorFrameTypesMsg);

        response.set_status(
            static_cast<::uvc_payload::Status>(aditof::Status::OK));

        break;
    }

    case uvc_payload::FunctionName::SET_FRAME_TYPE: {
        aditof::DepthSensorFrameType frameType;
        convertProtoMsgToDepthSensorFrameType(request.frame_type(), frameType);

        aditof::Status status = camDepthSensor->setFrameType(frameType);
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::READ_REGISTERS: {
        size_t length = static_cast<size_t>(request.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            request.func_bytes_param(0).c_str());
        uint16_t *data = new uint16_t[length];
        bool burst = static_cast<bool>(request.func_int32_param(1));

        aditof::Status status =
            camDepthSensor->readRegisters(address, data, length, burst);
        if (status == aditof::Status::OK) {
            response.add_bytes_payload(data, length * sizeof(uint16_t));
        }
        delete[] data;
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::WRITE_REGISTERS: {
        size_t length = static_cast<size_t>(request.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            request.func_bytes_param(0).c_str());
        const uint16_t *data = reinterpret_cast<const uint16_t *>(
            request.func_bytes_param(1).c_str());
        bool burst = static_cast<bool>(request.func_int32_param(1));

        aditof::Status status =
            camDepthSensor->writeRegisters(address, data, length, burst);
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::STORAGE_OPEN: {

        aditof::Status status = aditof::Status::OK;
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::STORAGE_READ: {

        size_t length = static_cast<size_t>(request.func_int32_param(0));
        const uint32_t address =
            static_cast<const uint32_t>(request.func_int32_param(1));
        uint8_t *data = new uint8_t[length];

        aditof::Status status = storages[0]->read(address, data, length);

        if (status == aditof::Status::OK) {
            response.add_bytes_payload(data, length * sizeof(uint8_t));
        }
        delete[] data;

        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::STORAGE_WRITE: {

        size_t length = static_cast<size_t>(request.func_int32_param(0));
        const uint32_t address =
            static_cast<const uint32_t>(request.func_int32_param(1));
        const uint8_t *data = reinterpret_cast<const uint8_t *>(
            request.func_bytes_param(0).c_str());

        aditof::Status status = storages[0]->write(address, data, length);

        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::STORAGE_CLOSE: {

        aditof::Status status = aditof::Status::OK;
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::STORAGE_READ_CAPACITY: {

        size_t data;

        aditof::Status status = storages[0]->getCapacity(data);

        if (status == aditof::Status::OK) {
            response.add_int32_payload(data);
        }

        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    default: {
        const std::string errorMsg(
            "Unknown function name set in the client request");
        LOG(ERROR) << errorMsg;
        response.set_message(errorMsg);
        response.set_status(
            static_cast<uvc_payload::Status>(aditof::Status::INVALID_ARGUMENT));
        break;
    }
    } // switch

    response.SerializeToString(&serverResponseBlob);

    string_response = (char *)malloc(serverResponseBlob.size()+1);
    memcpy(string_response, serverResponseBlob.c_str(), serverResponseBlob.size());
    string_response[serverResponseBlob.size()] = '\0';

    return string_response;
}
