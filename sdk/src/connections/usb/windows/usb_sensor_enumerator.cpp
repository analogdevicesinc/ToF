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
#include "usb_buffer.pb.h"
#include "connections/usb/usb_sensor_enumerator.h"
#include "connections/usb/usb_depth_sensor.h"
#include "connections/usb/usb_storage.h"
#include "connections/usb/usb_temperature_sensor.h"
#include "connections/usb/usb_utils.h"
#include "connections/usb/windows/usb_windows_utils.h"
#include "utils.h"

#include <glog/logging.h>

#include <atlstr.h>
#include <memory>
#include <strmif.h>

using namespace aditof;

UsbSensorEnumerator::~UsbSensorEnumerator() = default;

Status UsbSensorEnumerator::searchSensors() {
    using namespace std;
    Status status = Status::OK;

    LOG(INFO) << "Looking for USB connected sensors";

    HRESULT hr;

    hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

#if defined(ITOF)
    std::string devName("UVC Camera");
#else
    std::string devName("ADI TOF DEPTH SENSOR");
#endif
    ICreateDevEnum *DevEnum = NULL;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&DevEnum));
    if (FAILED(hr)) {
        LOG(ERROR) << "Create Device Enumeration Failed" << std::endl;
        return Status::GENERIC_ERROR;
    }

    IEnumMoniker *EnumMoniker = NULL;
    hr = DevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                        &EnumMoniker, 0);

    if (hr != S_OK) {
        DevEnum->Release();
        LOG(ERROR) << "Device Enumeration Error" << std::endl;
        return Status::GENERIC_ERROR;
    }

    IMoniker *Moniker = NULL;
    ULONG cFetched;
    while (EnumMoniker->Next(1, &Moniker, &cFetched) == S_OK) {
        IPropertyBag *PropBag;
        hr = Moniker->BindToStorage(0, 0, IID_PPV_ARGS(&PropBag));

        if (SUCCEEDED(hr)) {
            VARIANT varName;
            VariantInit(&varName);
            hr = PropBag->Read(L"FriendlyName", &varName, 0);

            if (SUCCEEDED(hr)) {
                std::string str(static_cast<LPCTSTR>(CString(varName.bstrVal)));
                if (str.find(devName) != std::string::npos) {
                    SensorInfo sInfo;
                    sInfo.driverPath = str;

                    DLOG(INFO) << "Found USB capture device: " << str;

                    std::string advertisedSensorData;

					IBaseFilter *pVideoInputFilter;

					HRESULT hr = Moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
						(void **)&pVideoInputFilter);
					if (!SUCCEEDED(hr)) {
						LOG(WARNING) << "Failed to bind video input filter";
						return Status::GENERIC_ERROR;
					}

                    // Query the sensors that are available on target

                    // Send request
                    usb_payload::ClientRequest requestMsg;
                    requestMsg.set_func_name(usb_payload::FunctionName::SEARCH_SENSORS);
                    std::string requestStr;
                    requestMsg.SerializeToString(&requestStr);
                    status = UsbWindowsUtils::uvcExUnitSendRequest(pVideoInputFilter, requestStr);
                    if (status != aditof::Status::OK) {
                        LOG(ERROR) << "Request to search for sensors failed";
                        return status;
                    }

                    // Read UVC gadget response
                    std::string responseStr;
                    status = UsbWindowsUtils::uvcExUnitGetResponse(pVideoInputFilter, responseStr);
                    if (status != aditof::Status::OK) {
                        LOG(ERROR) << "Request to search for sensors failed";
                        return status;
                    }
                    usb_payload::ServerResponse responseMsg;
                    bool parsed = responseMsg.ParseFromString(responseStr);
                    if (!parsed) {
                        LOG(ERROR) << "Failed to deserialize string containing UVC gadget response";
                        return aditof::Status::INVALID_ARGUMENT;
                    }

                    DLOG(INFO) << "Received the following message with "
                                  "available sensors from target: "
                               << responseMsg.DebugString();

                    if (responseMsg.status() != usb_payload::Status::OK) {
                        LOG(ERROR) << "Search for sensors operation failed on UVC gadget";
                        return static_cast<aditof::Status>(responseMsg.status());
                    }

                    // If request and response went well, extract data from response
                    m_sensorsInfo.emplace_back(sInfo);

                    m_storagesInfo.clear();
                    for (int i = 0; i < responseMsg.sensors_info().storages_size(); ++i) {
                        auto storage = responseMsg.sensors_info().storages(i);
                        m_storagesInfo.emplace_back(std::make_pair(storage.name(), storage.id()));
                    }

                    m_temperatureSensorsInfo.clear();
                    for (int i = 0; i < responseMsg.sensors_info().temp_sensors_size(); ++i) {
                        auto tempSensor = responseMsg.sensors_info().temp_sensors(i);
                        m_storagesInfo.emplace_back(std::make_pair(tempSensor.name(), tempSensor.id()));
                    }
                }
            }
            VariantClear(&varName);
            PropBag->Release();
            PropBag = NULL;
        }

        Moniker->Release();
        Moniker = NULL;
    }

    EnumMoniker->Release();
    DevEnum->Release();

    return status;
}

Status UsbSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<DepthSensorInterface>> &depthSensors) {

    depthSensors.clear();

    for (const auto &sInfo : m_sensorsInfo) {
        auto sensor = std::make_shared<UsbDepthSensor>(sInfo.driverPath);
        depthSensors.emplace_back(sensor);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getStorages(
    std::vector<std::shared_ptr<StorageInterface>> &storages) {
    storages.clear();

    for (const auto &nameAndId : m_storagesInfo) {
        auto storage =
            std::make_shared<UsbStorage>(nameAndId.first, nameAndId.second);
        storages.emplace_back(storage);
    }

    return Status::OK;
}

Status UsbSensorEnumerator::getTemperatureSensors(
    std::vector<std::shared_ptr<TemperatureSensorInterface>>
        &temperatureSensors) {

    temperatureSensors.clear();

    for (const auto &nameAndId : m_temperatureSensorsInfo) {
        auto tSensor = std::make_shared<UsbTemperatureSensor>(nameAndId.first,
                                                              nameAndId.second);
        temperatureSensors.emplace_back(tSensor);
    }

    return Status::OK;
}
