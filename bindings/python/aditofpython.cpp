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
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

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
        .def_readwrite("subelementSize",
                       &aditof::FrameDataDetails::subelementSize)
        .def_readwrite("subelementsPerElement",
                       &aditof::FrameDataDetails::subelementsPerElement)
        .def_readwrite("bytesCount", &aditof::FrameDataDetails::bytesCount);

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
        .def_readwrite("fx", &aditof::IntrinsicParameters::fx)
        .def_readwrite("fy", &aditof::IntrinsicParameters::fy)
        .def_readwrite("cx", &aditof::IntrinsicParameters::cx)
        .def_readwrite("cy", &aditof::IntrinsicParameters::cy)
        .def_readwrite("codx", &aditof::IntrinsicParameters::codx)
        .def_readwrite("cody", &aditof::IntrinsicParameters::cody)
        .def_readwrite("k1", &aditof::IntrinsicParameters::k1)
        .def_readwrite("k2", &aditof::IntrinsicParameters::k2)
        .def_readwrite("k3", &aditof::IntrinsicParameters::k3)
        .def_readwrite("k4", &aditof::IntrinsicParameters::k4)
        .def_readwrite("k5", &aditof::IntrinsicParameters::k5)
        .def_readwrite("k6", &aditof::IntrinsicParameters::k6)
        .def_readwrite("p2", &aditof::IntrinsicParameters::p2)
        .def_readwrite("p1", &aditof::IntrinsicParameters::p1);

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
            int nbDimensions =
                2 + (f.details.subelementsPerElement > 1 ? 1 : 0);
            // 2D configuration
            std::vector<ssize_t> shape = {
                static_cast<py::ssize_t>(f.details.height),
                static_cast<py::ssize_t>(f.details.width)};
            std::vector<ssize_t> strides = {
                static_cast<py::ssize_t>(f.details.subelementSize *
                                         f.details.subelementsPerElement *
                                         f.details.width),
                static_cast<py::ssize_t>(f.details.subelementSize *
                                         f.details.subelementsPerElement)};

            // Additions for a 3D configuration
            if (nbDimensions == 3) {
                shape.emplace_back(
                    static_cast<py::ssize_t>(f.details.subelementsPerElement));
                strides.emplace_back(
                    static_cast<py::ssize_t>(f.details.subelementSize));
            }

            std::string format;
            if (f.details.subelementSize == 1) {
                format = "B";
            } else if (f.details.subelementSize == 2) {
                format = "H";
            }

            return py::buffer_info(
                f.pData, static_cast<py::ssize_t>(f.details.subelementSize),
                format, nbDimensions, shape, strides);
        });

    // ADI Time of Flight API

    // System
    py::class_<aditof::System>(m, "System")
        .def(py::init<>())
        .def(
            "getCameraList",
            [](aditof::System &system, py::list cameras, py::str ip) {
                std::vector<std::shared_ptr<aditof::Camera>> cameraList;
                aditof::Status status = system.getCameraList(cameraList, ip);

                for (const auto &cam : cameraList) {
                    cameras.append(cam);
                }

                return status;
            },
            py::arg("cameras"), py::arg("ip"));

    // Camera
    py::class_<aditof::Camera, std::shared_ptr<aditof::Camera>>(m, "Camera")
        .def("initialize", &aditof::Camera::initialize,
             py::arg("configFilepath") = "")
        .def("start", &aditof::Camera::start)
        .def("stop", &aditof::Camera::stop)
        .def("setMode", &aditof::Camera::setMode, py::arg("mode"),
             py::arg("modeFilename") = "")
        .def(
            "getAvailableModes",
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
        .def(
            "getAvailableFrameTypes",
            [](const aditof::Camera &camera, py::list types) {
                std::vector<std::string> typeList;
                aditof::Status status = camera.getAvailableFrameTypes(typeList);

                for (const auto &type : typeList)
                    types.append(type);

                return status;
            },
            py::arg("availableFrameTypes"))
        .def("requestFrame", &aditof::Camera::requestFrame, py::arg("frame"),
             py::arg("cb") = nullptr)
        .def("getDetails", &aditof::Camera::getDetails, py::arg("details"))
        .def(
            "getAvailableControls",
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
        .def("adsd3500SetToggleMode", &aditof::Camera::adsd3500SetToggleMode,
             py::arg("mode"))
        .def("adsd3500ToggleFsync", &aditof::Camera::adsd3500ToggleFsync)
        .def("adsd3500SetABinvalidationThreshold",
             &aditof::Camera::adsd3500SetABinvalidationThreshold,
             py::arg("threshold"))
        .def("adsd3500GetABinvalidationThreshold",
             [](aditof::Camera &camera) {
                 int threshold;
                 aditof::Status status =
                     camera.adsd3500GetABinvalidationThreshold(threshold);
                 return std::make_pair(status, threshold);
             })
        .def("adsd3500SetConfidenceThreshold",
             &aditof::Camera::adsd3500SetConfidenceThreshold,
             py::arg("threshold"))
        .def("adsd3500GetConfidenceThreshold",
             [](aditof::Camera &camera) {
                 int threshold;
                 aditof::Status status =
                     camera.adsd3500GetConfidenceThreshold(threshold);
                 return std::make_pair(status, threshold);
             })
        .def("adsd3500SetJBLFfilterEnableState",
             &aditof::Camera::adsd3500SetJBLFfilterEnableState,
             py::arg("enable"))
        .def("adsd3500GetJBLFfilterEnableState",
             [](aditof::Camera &camera) {
                 bool enableState;
                 aditof::Status status =
                     camera.adsd3500GetJBLFfilterEnableState(enableState);
                 return std::make_pair(status, enableState);
             })
        .def("adsd3500SetJBLFfilterSize",
             &aditof::Camera::adsd3500SetJBLFfilterSize, py::arg("size"))
        .def("adsd3500GetJBLFfilterSize",
             [](aditof::Camera &camera) {
                 int size;
                 aditof::Status status = camera.adsd3500GetJBLFfilterSize(size);
                 return std::make_pair(status, size);
             })
        .def("adsd3500SetRadialThresholdMin",
             &aditof::Camera::adsd3500SetRadialThresholdMin,
             py::arg("threshold"))
        .def("adsd3500GetRadialThresholdMin",
             [](aditof::Camera &camera) {
                 int threshold;
                 aditof::Status status =
                     camera.adsd3500GetRadialThresholdMin(threshold);
                 return std::make_pair(status, threshold);
             })
        .def("adsd3500SetRadialThresholdMax",
             &aditof::Camera::adsd3500SetRadialThresholdMax,
             py::arg("threshold"))
        .def("adsd3500GetRadialThresholdMax",
             [](aditof::Camera &camera) {
                 int threshold;
                 aditof::Status status =
                     camera.adsd3500GetRadialThresholdMax(threshold);
                 return std::make_pair(status, threshold);
             })
        .def("adsd3500SetMIPIOutputSpeed",
             &aditof::Camera::adsd3500SetMIPIOutputSpeed, py::arg("speed"))
        .def("adsd3500GetMIPIOutputSpeed",
             [](aditof::Camera &camera) {
                 uint16_t speed;
                 aditof::Status status =
                     camera.adsd3500GetMIPIOutputSpeed(speed);
                 return std::make_pair(status, speed);
             })
        .def("adsd3500GetImagerErrorCode",
             [](aditof::Camera &camera) {
                 uint16_t errcode;
                 aditof::Status status =
                     camera.adsd3500GetImagerErrorCode(errcode);
                 return std::make_pair(status, errcode);
             })
        .def("adsd3500SetVCSELDelay", &aditof::Camera::adsd3500SetVCSELDelay,
             py::arg("delay"))
        .def("adsd3500GetVCSELDelay",
             [](aditof::Camera &camera) {
                 uint16_t delay;
                 aditof::Status status = camera.adsd3500GetVCSELDelay(delay);
                 return std::make_pair(status, delay);
             })
        .def("adsd3500SetJBLFMaxEdgeThreshold",
             &aditof::Camera::adsd3500SetJBLFMaxEdgeThreshold,
             py::arg("threshold"))
        .def("adsd3500SetJBLFABThreshold",
             &aditof::Camera::adsd3500SetJBLFABThreshold, py::arg("threshold"))
        .def("adsd3500SetJBLFGaussianSigma",
             &aditof::Camera::adsd3500SetJBLFGaussianSigma, py::arg("value"))
        .def("adsd3500GetJBLFGaussianSigma",
             [](aditof::Camera &camera) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetJBLFGaussianSigma(value);
                 return std::make_pair(status, value);
             })
        .def("adsd3500SetJBLFExponentialTerm",
             &aditof::Camera::adsd3500SetJBLFExponentialTerm, py::arg("value"))
        .def("adsd3500GetJBLFExponentialTerm",
             [](aditof::Camera &camera) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetJBLFExponentialTerm(value);
                 return std::make_pair(status, value);
             })
        .def("adsd3500SetFrameRate", &aditof::Camera::adsd3500SetFrameRate,
             py::arg("value"))
        .def("adsd3500GetFrameRate",
             [](aditof::Camera &camera) {
                 uint16_t framerate;
                 aditof::Status status = camera.adsd3500GetFrameRate(framerate);
                 return std::make_pair(status, framerate);
             })
        .def("adsd3500SetEnableEdgeConfidence",
             &aditof::Camera::adsd3500SetEnableEdgeConfidence, py::arg("value"))
        .def("adsd3500GetTemperatureCompensationStatus",
             [](aditof::Camera &camera) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetTemperatureCompensationStatus(value);
                 return std::make_pair(status, value);
             })
        .def("adsd3500SetEnablePhaseInvalidation",
             &aditof::Camera::adsd3500SetEnablePhaseInvalidation,
             py::arg("value"))
        .def("adsd3500SetEnableTemperatureCompensation",
             &aditof::Camera::adsd3500SetEnableTemperatureCompensation,
             py::arg("value"))
        .def("adsd3500SetEnableEmbeddedHeaderinAB",
             &aditof::Camera::adsd3500SetEnableEmbeddedHeaderinAB,
             py::arg("value"))
        .def("adsd3500GetEnableEmbeddedHeaderinAB",
             [](aditof::Camera &camera) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetEnableEmbeddedHeaderinAB(value);
                 return std::make_pair(status, value);
             })
        .def("adsd3500SetGenericTemplate",
             &aditof::Camera::adsd3500SetGenericTemplate, py::arg("reg"),
             py::arg("value"))
        .def("adsd3500GetGenericTemplate",
             [](aditof::Camera &camera, uint16_t reg) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetGenericTemplate(reg, value);
                 return std::make_pair(status, value);
             });

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

                frame.getData(dataType,
                              reinterpret_cast<uint16_t **>(&f.pData));
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
        .def(
            "getAvailableFrameTypes",
            [](aditof::DepthSensorInterface &device, py::list types) {
                std::vector<aditof::DepthSensorFrameType> typeList;
                aditof::Status status = device.getAvailableFrameTypes(typeList);

                for (const auto &type : typeList)
                    types.append(type);

                return status;
            },
            py::arg("types"))
        .def("setFrameType", &aditof::DepthSensorInterface::setFrameType,
             py::arg("details"))
        .def(
            "program",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint8_t> firmware, size_t size) {
                py::buffer_info buffInfo = firmware.request();
                uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                return device.program(ptr, size);
            },
            py::arg("firmware"), py::arg("size"))
        .def(
            "getFrame",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint16_t> buffer) {
                py::buffer_info buffInfo = buffer.request(true);
                uint16_t *ptr = static_cast<uint16_t *>(buffInfo.ptr);

                return device.getFrame(ptr);
            },
            py::arg("buffer"))
        .def(
            "regread",
            [](aditof::DepthSensorInterface &device, uint16_t address) {
                uint16_t addrPtr[1], dataPtr[1];
                addrPtr[0] = address;
                device.readRegisters(addrPtr, dataPtr, 1);
                return dataPtr[0];
            },
            py::arg("address"))
        .def(
            "regwrite",
            [](aditof::DepthSensorInterface &device, uint16_t address,
               uint16_t data) {
                uint16_t addrPtr[1], dataPtr[1];
                addrPtr[0] = address;
                dataPtr[0] = data;
                return device.writeRegisters(addrPtr, dataPtr, 1);
            },
            py::arg("address"), py::arg("data"))
        .def(
            "regwriteburst",
            [](aditof::DepthSensorInterface &device, uint16_t address,
               py::array_t<uint16_t> data, const std::string &incr) {
                uint16_t addrPtr[1];
                addrPtr[0] = address;
                if (incr.compare("increment") == 0)
                    addrPtr[0] |= 0x4000;
                py::buffer_info dataBuffInfo = data.request();
                uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);
                return device.writeRegisters(addrPtr, dataPtr, data.size());
            },
            py::arg("address"), py::arg("data"), py::arg("increment"))
        .def(
            "readRegisters",
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
        .def(
            "writeRegisters",
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

    //SDK version
    m.def("getApiVersion", &aditof::getApiVersion);
    m.def("getBranchVersion", &aditof::getBranchVersion);
    m.def("getCommitVersion", &aditof::getCommitVersion);
}
