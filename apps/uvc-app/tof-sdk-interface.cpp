#include "tof-sdk-interface.h"
#include "buffer.pb.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include "reset.h"

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_definitions.h>
#include <aditof/sensor_enumerator_factory.h>
#include <aditof/sensor_enumerator_interface.h>

/* Available sensors */
std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;

/* Version information */
std::string kernelversion;
std::string ubootversion;
std::string sdversion;

std::string connectionType = "USB";

int init_tof_sdk(char *cap_dev_path) {
    auto sensorsEnumerator =
        aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorsEnumerator) {
        DLOG(ERROR) << "Failed to construct a sensors enumerator!\n";
        return 1;
    }

    sensorsEnumerator->searchSensors();
    sensorsEnumerator->getDepthSensors(depthSensors);
    sensorsEnumerator->getKernelVersion(kernelversion);
    sensorsEnumerator->getUbootVersion(ubootversion);
    sensorsEnumerator->getSdVersion(sdversion);

    if (depthSensors.size() < 1) {
        DLOG(ERROR) << "No camera sensor are available!\n";
        return 1;
    }

    camDepthSensor = depthSensors[0];
    camDepthSensor->setHostConnectionType(connectionType);
    camDepthSensor->open();

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

void handleClientRequest(const char *in_buf, const size_t in_len,
                         char **out_buf, size_t *out_len) {
    uvc_payload::ClientRequest request;
    uvc_payload::ServerResponse response;
    std::string serverResponseBlob;

    request.ParseFromString(std::string(in_buf, in_len));

    switch (request.func_name()) {

    case uvc_payload::FunctionName::SEARCH_SENSORS: {
        // Depth sensor
        if (depthSensors.size() < 1) {
            response.set_message("No depth sensors are available");
            response.set_status(::uvc_payload::Status::UNREACHABLE);
            break;
        }

        auto pbSensorsInfo = response.mutable_sensors_info();

        std::string name;
        camDepthSensor->getName(name);
        auto pbDepthSensorInfo = pbSensorsInfo->mutable_image_sensors();
        pbDepthSensorInfo->set_name(name);

        auto cardVersion = response.mutable_card_image_version();

        cardVersion->set_kernelversion(kernelversion);
        cardVersion->set_ubootversion(ubootversion);
        cardVersion->set_sdversion(sdversion);

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
    case uvc_payload::FunctionName::GET_AVAILABLE_CONTROLS: {
        std::vector<std::string> controls;
        aditof::Status status = camDepthSensor->getAvailableControls(controls);
        for (const auto &control : controls) {
            response.add_strings_payload(control);
        }
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::SET_CONTROL: {
        std::string controlName = request.func_strings_param(0);
        std::string controlValue = request.func_strings_param(1);
        aditof::Status status =
            camDepthSensor->setControl(controlName, controlValue);
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::GET_CONTROL: {
        std::string controlName = request.func_strings_param(0);
        std::string controlValue;
        aditof::Status status =
            camDepthSensor->getControl(controlName, controlValue);
        response.add_strings_payload(controlValue);
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::START: {
        aditof::Status status = camDepthSensor->start();
        response.set_status(static_cast<::uvc_payload::Status>(status));

        break;
    }

    case uvc_payload::FunctionName::ADSD3500_READ_CMD: {
        uint16_t cmd = static_cast<uint32_t>(request.func_int32_param(0));
        uint16_t *data = new uint16_t;
        unsigned int usDelay =
            static_cast<unsigned int>(request.func_int32_param(1));

        aditof::Status status =
            camDepthSensor->adsd3500_read_cmd(cmd, data, usDelay);

        if (status == aditof::Status::OK) {
            response.add_bytes_payload(data, sizeof(uint16_t));
        }
        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_WRITE_CMD: {
        uint16_t cmd = static_cast<uint16_t>(request.func_int32_param(0));
        uint16_t data;

        memcpy(&data, request.func_bytes_param(0).c_str(), sizeof(uint16_t));

        aditof::Status status = camDepthSensor->adsd3500_write_cmd(cmd, data);

        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_READ_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(request.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(request.func_int32_param(1));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, request.func_bytes_param(0).c_str(), 4 * sizeof(uint8_t));

        aditof::Status status =
            camDepthSensor->adsd3500_read_payload_cmd(cmd, data, payload_len);
        if (status == aditof::Status::OK) {
            response.add_bytes_payload(data, payload_len * sizeof(uint8_t));
        }
        delete[] data;
        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_READ_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(request.func_int32_param(0));
        uint8_t *data = new uint8_t[payload_len];

        aditof::Status status =
            camDepthSensor->adsd3500_read_payload(data, payload_len);
        if (status == aditof::Status::OK) {
            response.add_bytes_payload(data, payload_len * sizeof(uint8_t));
        }
        delete[] data;

        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_WRITE_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(request.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(request.func_int32_param(1));
        const uint8_t *data = reinterpret_cast<const uint8_t *>(
            request.func_bytes_param(0).c_str());
        aditof::Status status = camDepthSensor->adsd3500_write_payload_cmd(
            cmd, (uint8_t *)data, payload_len);

        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_WRITE_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(request.func_int32_param(0));
        const uint8_t *data = reinterpret_cast<const uint8_t *>(
            request.func_bytes_param(0).c_str());
        aditof::Status status = camDepthSensor->adsd3500_write_payload(
            (uint8_t *)data, payload_len);

        response.set_status(static_cast<::uvc_payload::Status>(status));
        break;
    }

    case uvc_payload::FunctionName::ADSD3500_RESET: {

        modeChanged = 1;

        response.set_status(
            static_cast<::uvc_payload::Status>(aditof::Status::OK));
        break;
    }

    case uvc_payload::FunctionName::INIT_TARGET_DEPTH_COMPUTE: {
        uint16_t iniFileLength =
            static_cast<uint16_t>(request.func_int32_param(0));
        uint16_t calDataLength =
            static_cast<uint16_t>(request.func_int32_param(1));
        uint8_t *iniFile = new uint8_t[iniFileLength];
        uint8_t *calData = new uint8_t[calDataLength];

        memcpy(iniFile, request.func_bytes_param(0).c_str(), iniFileLength);
        memcpy(calData, request.func_bytes_param(1).c_str(), calDataLength);

        aditof::Status status = camDepthSensor->initTargetDepthCompute(
            iniFile, iniFileLength, calData, calDataLength);
        response.set_status(static_cast<::uvc_payload::Status>(status));

        delete[] iniFile;
        delete[] calData;

        break;
    }

    case uvc_payload::FunctionName::PROCESS_FRAME: {

        aditof::Status status = aditof::Status::OK;
        status = camDepthSensor->getFrame(nullptr);
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

    *out_buf = (char *)malloc(serverResponseBlob.size());
    memcpy(*out_buf, serverResponseBlob.c_str(), serverResponseBlob.size());
    *out_len = serverResponseBlob.size();

    return;
}
