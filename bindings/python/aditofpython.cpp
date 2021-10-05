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
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <aditof/aditof.h>

namespace py = pybind11;

PYBIND11_MODULE(aditofpython, m) {

    m.doc() = "ADI Time Of Flight python extensions";

    // General declarations

    py::enum_<aditof::Status>(m, "Status")
        .value("Ok", aditof::Status::OK)
        .value("Busy", aditof::Status::BUSY)
        .value("Unreachable", aditof::Status::UNREACHABLE)
        .value("InvalidArgument", aditof::Status::INVALID_ARGUMENT)
        .value("Unavailable", aditof::Status::UNAVAILABLE)
        .value("GenericError", aditof::Status::GENERIC_ERROR);

    // Frame declarations

    py::class_<aditof::FrameDataDetails>(m, "FrameDataDetails")
        .def(py::init<>())
        .def_readwrite("type", &aditof::FrameDataDetails::type)
        .def_readwrite("width", &aditof::FrameDataDetails::width)
        .def_readwrite("height", &aditof::FrameDataDetails::height)
        .def_readwrite("subelementSize", &aditof::FrameDataDetails::subelementSize)
        .def_readwrite("subelementsPerElement", &aditof::FrameDataDetails::subelementsPerElement);

    py::class_<aditof::FrameDetails>(m, "FrameDetails")
        .def(py::init<>())
        .def_readwrite("type", &aditof::FrameDetails::type)
        .def_readwrite("dataDetails", &aditof::FrameDetails::dataDetails)
        .def_readwrite("cameraMode", &aditof::FrameDetails::cameraMode);

    // Camera declarations

    py::enum_<aditof::ConnectionType>(m, "ConnectionType")
        .value("Usb", aditof::ConnectionType::USB)
        .value("Network", aditof::ConnectionType::NETWORK)
        .value("OnTarget", aditof::ConnectionType::ON_TARGET);

    py::class_<aditof::IntrinsicParameters>(m, "IntrinsicParameters")
        .def(py::init<>())
        .def_readwrite("cameraMatrix",
                       &aditof::IntrinsicParameters::cameraMatrix)
        .def_readwrite("distCoeffs", &aditof::IntrinsicParameters::distCoeffs)
        .def_readwrite("pixelWidth", &aditof::IntrinsicParameters::pixelWidth)
        .def_readwrite("pixelHeight",
                       &aditof::IntrinsicParameters::pixelHeight);

    py::class_<aditof::CameraDetails>(m, "CameraDetails")
        .def(py::init<>())
        .def_readwrite("cameraId", &aditof::CameraDetails::cameraId)
        .def_readwrite("mode", &aditof::CameraDetails::mode)
        .def_readwrite("frameType", &aditof::CameraDetails::frameType)
        .def_readwrite("connection", &aditof::CameraDetails::connection)
        .def_readwrite("intrinsics", &aditof::CameraDetails::intrinsics)
        .def_readwrite("minDepth", &aditof::CameraDetails::minDepth)
        .def_readwrite("maxDepth", &aditof::CameraDetails::maxDepth)
        .def_readwrite("bitCount", &aditof::CameraDetails::bitCount);

    // Sensors declarations

    py::class_<aditof::SensorDetails>(m, "SensorDetails")
        .def(py::init<>())
        .def_readwrite("sensorName", &aditof::SensorDetails::sensorName)
        .def_readwrite("connectionType",
                       &aditof::SensorDetails::connectionType);

    py::class_<aditof::DepthSensorFrameContent>(m, "DepthSensorFrameContent")
        .def(py::init<>())
        .def_readwrite("type", &aditof::DepthSensorFrameContent::type)
        .def_readwrite("width", &aditof::DepthSensorFrameContent::width)
        .def_readwrite("height", &aditof::DepthSensorFrameContent::height);

    py::class_<aditof::DepthSensorFrameType>(m, "DepthSensorFrameType")
        .def(py::init<>())
        .def_readwrite("type", &aditof::DepthSensorFrameType::type)
        .def_readwrite("content", &aditof::DepthSensorFrameType::content)
        .def_readwrite("width", &aditof::DepthSensorFrameType::width)
        .def_readwrite("height", &aditof::DepthSensorFrameType::height);

    // Helpers

    struct frameData {
        void *pData;
        aditof::FrameDataDetails details;
    };

    py::class_<frameData>(m, "frameData", py::buffer_protocol())
        .def(py::init<>())
        .def_buffer([](const frameData &f) -> py::buffer_info {
            int nbDimensions = 2 + (f.details.subelementsPerElement > 1 ? 1 : 0);
            // 2D configuration
            std::vector<ssize_t> shape = {static_cast<py::ssize_t>(f.details.height),
                static_cast<py::ssize_t>(f.details.width)};
            std::vector<ssize_t> strides = {static_cast<py::ssize_t>(f.details.subelementSize * f.details.subelementsPerElement * f.details.width),
                static_cast<py::ssize_t>(f.details.subelementSize * f.details.subelementsPerElement)};

            // Additions for a 3D configuration
            if (nbDimensions == 3) {
                shape.emplace_back(static_cast<py::ssize_t>(f.details.subelementsPerElement));
                strides.emplace_back(static_cast<py::ssize_t>(f.details.subelementSize));
            }

            std::string format;
            if (f.details.subelementSize == 1) {
                format = "B";
            } else if (f.details.subelementSize == 2) {
                format = "H";
            }

            return py::buffer_info(
                f.pData,
                static_cast<py::ssize_t>(f.details.subelementSize),
                format,
                nbDimensions,
                shape,
                strides);
        });

    // ADI Time of Flight API

    // System
    py::class_<aditof::System>(m, "System")
        .def(py::init<>())
        .def("getCameraList",
             [](aditof::System &system, py::list cameras) {
                 std::vector<std::shared_ptr<aditof::Camera>> cameraList;
                 aditof::Status status = system.getCameraList(cameraList);

                 for (const auto &cam : cameraList) {
                     cameras.append(cam);
                 }

                 return status;
             },
             py::arg("cameras"))
        .def("getCameraListAtIp",
             [](aditof::System &system, py::list cameras, py::str ip) {
                 std::vector<std::shared_ptr<aditof::Camera>> cameraList;
                 aditof::Status status =
                     system.getCameraListAtIp(cameraList, ip);

                 for (const auto &cam : cameraList) {
                     cameras.append(cam);
                 }

                 return status;
             },
             py::arg("cameras"), py::arg("ip"));

    // Camera
    py::class_<aditof::Camera, std::shared_ptr<aditof::Camera>>(m, "Camera")
        .def("initialize", &aditof::Camera::initialize)
        .def("start", &aditof::Camera::start)
        .def("stop", &aditof::Camera::stop)
        .def("setMode", &aditof::Camera::setMode, py::arg("mode"),
             py::arg("modeFilename") = "")
        .def("getAvailableModes",
             [](const aditof::Camera &camera, py::list modes) {
                 std::vector<std::string> modeList;
                 aditof::Status status = camera.getAvailableModes(modeList);

                 for (const auto &mode : modeList)
                     modes.append(mode);

                 return status;
             },
             py::arg("availableModes"))
        .def("setFrameType", &aditof::Camera::setFrameType,
             py::arg("frameType"))
        .def("getAvailableFrameTypes",
             [](const aditof::Camera &camera, py::list types) {
                 std::vector<std::string> typeList;
                 aditof::Status status =
                     camera.getAvailableFrameTypes(typeList);

                 for (const auto &type : typeList)
                     types.append(type);

                 return status;
             },
             py::arg("availableFrameTypes"))
        .def("requestFrame", &aditof::Camera::requestFrame, py::arg("frame"),
             py::arg("cb") = nullptr)
        .def("getDetails", &aditof::Camera::getDetails, py::arg("details"))
        .def("getAvailableControls",
             [](const aditof::Camera &camera, py::list controls) {
                 std::vector<std::string> controlsList;
                 aditof::Status status =
                     camera.getAvailableControls(controlsList);

                 for (const auto &control : controlsList)
                     controls.append(control);

                 return status;
             },
             py::arg("controls"))
        .def("setControl", &aditof::Camera::setControl, py::arg("control"),
             py::arg("value"))
        .def("getControl", &aditof::Camera::getControl, py::arg("control"),
             py::arg("value"))
        .def("getSensor", &aditof::Camera::getSensor)
        .def("getEeproms",
             [](aditof::Camera &camera, py::list eeproms) {
                 std::vector<std::shared_ptr<aditof::StorageInterface>>
                     eepromList;
                 aditof::Status status = camera.getEeproms(eepromList);

                 for (const auto &e : eepromList)
                     eeproms.append(e);

                 return status;
             },
             py::arg("eeproms"))
        .def(
            "getTemperatureSensors",
            [](aditof::Camera &camera, py::list tempSensors) {
                std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
                    sensorList;
                aditof::Status status =
                    camera.getTemperatureSensors(sensorList);

                for (const auto &s : sensorList)
                    tempSensors.append(s);

                return status;
            },
            py::arg("tempSensors"));

    // Frame
    py::class_<aditof::Frame>(m, "Frame")
        .def(py::init<>())
        .def("setDetails", &aditof::Frame::setDetails, py::arg("details"))
        .def("getDetails", &aditof::Frame::getDetails, py::arg("details"))
        .def("getDataDetails", &aditof::Frame::getDataDetails,
             py::arg("dataType"), py::arg("dataDetails"))
        .def(
            "getData",
            [](aditof::Frame &frame, const std::string &dataType) -> frameData {
                frameData f;

                frame.getData(dataType, reinterpret_cast<uint16_t **>(&f.pData));
                frame.getDataDetails(dataType, f.details);

                return f;
            },
            py::arg("dataType"));

    // DepthSensorInterface
    py::class_<aditof::DepthSensorInterface,
               std::shared_ptr<aditof::DepthSensorInterface>>(
        m, "DepthSensorInterface")
        .def("open", &aditof::DepthSensorInterface::open)
        .def("start", &aditof::DepthSensorInterface::start)
        .def("stop", &aditof::DepthSensorInterface::stop)
        .def("getAvailableFrameTypes",
             [](aditof::DepthSensorInterface &device, py::list types) {
                 std::vector<aditof::DepthSensorFrameType> typeList;
                 aditof::Status status =
                     device.getAvailableFrameTypes(typeList);

                 for (const auto &type : typeList)
                     types.append(type);

                 return status;
             },
             py::arg("types"))
        .def("setFrameType", &aditof::DepthSensorInterface::setFrameType,
             py::arg("details"))
        .def("program",
             [](aditof::DepthSensorInterface &device,
                py::array_t<uint8_t> firmware, size_t size) {
                 py::buffer_info buffInfo = firmware.request();
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return device.program(ptr, size);
             },
             py::arg("firmware"), py::arg("size"))
        .def("getFrame",
             [](aditof::DepthSensorInterface &device,
                py::array_t<uint16_t> buffer) {
                 py::buffer_info buffInfo = buffer.request(true);
                 uint16_t *ptr = static_cast<uint16_t *>(buffInfo.ptr);

                 return device.getFrame(ptr);
             },
             py::arg("buffer"))
        .def("regread",
             [](aditof::DepthSensorInterface &device,
                uint16_t address) {
                  uint16_t addrPtr[1], dataPtr[1];
                  addrPtr[0] = address;
                  device.readRegisters(addrPtr, dataPtr, 1);
                  return dataPtr[0];
             },
             py::arg("address"))
        .def("regwrite",
             [](aditof::DepthSensorInterface &device,
                uint16_t address, uint16_t data) {
                   uint16_t addrPtr[1], dataPtr[1];
                   addrPtr[0] = address;
                   dataPtr[0] = data;
                   return device.writeRegisters(addrPtr, dataPtr, 1);
             },
             py::arg("address"), py::arg("data"))
        .def("regwriteburst",
             [](aditof::DepthSensorInterface &device,
                uint16_t address, py::array_t<uint16_t> data,
                const std::string &incr) {
                  uint16_t addrPtr[1];
                  addrPtr[0] = address;
                  if (incr.compare("increment") == 0)
                    addrPtr[0] |= 0x4000;
                  py::buffer_info dataBuffInfo = data.request();
                  uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);
                  return device.writeRegisters(addrPtr, dataPtr, data.size());
             },
             py::arg("address"), py::arg("data"), py::arg("increment"))
        .def("readRegisters",
             [](aditof::DepthSensorInterface &device,
                py::array_t<uint16_t> address, py::array_t<uint16_t> data,
                size_t length) {
                 py::buffer_info addrBuffInfo = address.request();
                 uint16_t *addrPtr = static_cast<uint16_t *>(addrBuffInfo.ptr);

                 py::buffer_info dataBuffInfo = data.request(true);
                 uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);

                 return device.readRegisters(addrPtr, dataPtr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("writeRegisters",
             [](aditof::DepthSensorInterface &device,
                py::array_t<uint16_t> address, py::array_t<uint16_t> data,
                size_t length) {
                 py::buffer_info addrBuffInfo = address.request();
                 uint16_t *addrPtr = static_cast<uint16_t *>(addrBuffInfo.ptr);

                 py::buffer_info dataBuffInfo = data.request();
                 uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);

                 return device.writeRegisters(addrPtr, dataPtr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"));

    // StorageInterface
    py::class_<aditof::StorageInterface,
               std::shared_ptr<aditof::StorageInterface>>(m, "StorageInterface")
        .def("open", &aditof::StorageInterface::open)
        .def("read",
             [](aditof::StorageInterface &eeprom, uint32_t address,
                py::array_t<uint8_t> data, size_t length) {
                 py::buffer_info buffInfo = data.request(true);
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return eeprom.read(address, ptr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("write",
             [](aditof::StorageInterface &eeprom, uint32_t address,
                py::array_t<uint8_t> data, size_t length) {
                 py::buffer_info buffInfo = data.request();
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return eeprom.write(address, ptr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("close", &aditof::StorageInterface::close)
        .def("getName", [](aditof::StorageInterface &eeprom) {
            std::string n;
            eeprom.getName(n);

            return n;
        });

    // TemperatureSensorInterface
    py::class_<aditof::TemperatureSensorInterface,
               std::shared_ptr<aditof::TemperatureSensorInterface>>(
        m, "TemperatureSensorInterface")
        .def("open", &aditof::TemperatureSensorInterface::open)
        .def("read",
             [](aditof::TemperatureSensorInterface &sensor,
                py::list temperature) {
                 float temp;
                 aditof::Status status = sensor.read(temp);
                 if (status == aditof::Status::OK) {
                     temperature.append(temp);
                 }
                 return status;
             },
             py::arg("temperature"))
        .def("close", &aditof::TemperatureSensorInterface::close)
        .def("getName", [](aditof::TemperatureSensorInterface &sensor) {
            std::string n;
            sensor.getName(n);

            return n;
        });
}
