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

    py::enum_<aditof::Adsd3500Status>(m, "Adsd3500Status")
        .value("OK", aditof::Adsd3500Status::OK)
        .value("Invalid_Mode", aditof::Adsd3500Status::INVALID_MODE)
        .value("Invalid_JBLF_Filter_Size",
               aditof::Adsd3500Status::INVALID_JBLF_FILTER_SIZE)
        .value("Unsupported_Command",
               aditof::Adsd3500Status::UNSUPPORTED_COMMAND)
        .value("Invalid_Memory_Region",
               aditof::Adsd3500Status::INVALID_MEMORY_REGION)
        .value("Invalid_Firmware_Crc",
               aditof::Adsd3500Status::INVALID_FIRMWARE_CRC)
        .value("Invalid_Imager", aditof::Adsd3500Status::INVALID_IMAGER)
        .value("Invalid_Ccb", aditof::Adsd3500Status::INVALID_CCB)
        .value("Flash_Header_Parse_Error",
               aditof::Adsd3500Status::FLASH_HEADER_PARSE_ERROR)
        .value("Flash_File_Parse_Error",
               aditof::Adsd3500Status::FLASH_FILE_PARSE_ERROR)
        .value("Spim_Error", aditof::Adsd3500Status::SPIM_ERROR)
        .value("Invalid_Chipid", aditof::Adsd3500Status::INVALID_CHIPID)
        .value("Imager_Communication_Error",
               aditof::Adsd3500Status::IMAGER_COMMUNICATION_ERROR)
        .value("Imager_Boot_Failure",
               aditof::Adsd3500Status::IMAGER_BOOT_FAILURE)
        .value("Firmware_Update_Complete",
               aditof::Adsd3500Status::FIRMWARE_UPDATE_COMPLETE)
        .value("Nvm_Write_Complete", aditof::Adsd3500Status::NVM_WRITE_COMPLETE)
        .value("Imager_Error", aditof::Adsd3500Status::IMAGER_ERROR)
        .value("Unknown_Error_Id", aditof::Adsd3500Status::UNKNOWN_ERROR_ID);

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
        .def_readwrite("cameraMode", &aditof::FrameDetails::cameraMode)
        .def_readwrite("width", &aditof::FrameDetails::width)
        .def_readwrite("height", &aditof::FrameDetails::height)
        .def_readwrite("totalCaptures", &aditof::FrameDetails::totalCaptures)
        .def_readwrite("passiveIRCaptured",
                       &aditof::FrameDetails::passiveIRCaptured);

    py::class_<aditof::Metadata>(m, "Metadata")
        .def(py::init<>())
        .def_readwrite("width", &aditof::Metadata::width)
        .def_readwrite("height", &aditof::Metadata::height)
        .def_readwrite("outputConfiguration",
                       &aditof::Metadata::outputConfiguration)
        .def_readwrite("bitsInDepth", &aditof::Metadata::bitsInDepth)
        .def_readwrite("bitsInAb", &aditof::Metadata::bitsInAb)
        .def_readwrite("bitsInConfidence", &aditof::Metadata::bitsInConfidence)
        .def_readwrite("invalidPhaseValue",
                       &aditof::Metadata::invalidPhaseValue)
        .def_readwrite("frequencyIndex", &aditof::Metadata::frequencyIndex)
        .def_readwrite("abFrequencyIndex", &aditof::Metadata::abFrequencyIndex)
        .def_readwrite("frameNumber", &aditof::Metadata::frameNumber)
        .def_readwrite("imagerMode", &aditof::Metadata::imagerMode)
        .def_readwrite("numberOfPhases", &aditof::Metadata::numberOfPhases)
        .def_readwrite("numberOfFrequencies",
                       &aditof::Metadata::numberOfFrequencies)
        .def_readwrite("xyzEnabled", &aditof::Metadata::xyzEnabled)
        .def_readwrite("elapsedTimeFractionalValue",
                       &aditof::Metadata::elapsedTimeFractionalValue)
        .def_readwrite("elapsedTimeSecondsValue",
                       &aditof::Metadata::elapsedTimeSecondsValue)
        .def_readwrite("elapsedTimeSecondsValue",
                       &aditof::Metadata::elapsedTimeSecondsValue)
        .def_readwrite("sensorTemperature",
                       &aditof::Metadata::sensorTemperature)
        .def_readwrite("laserTemperature", &aditof::Metadata::laserTemperature);

    // Camera declarations

    py::enum_<aditof::ConnectionType>(m, "ConnectionType")
        .value("Usb", aditof::ConnectionType::USB)
        .value("Network", aditof::ConnectionType::NETWORK)
        .value("OnTarget", aditof::ConnectionType::ON_TARGET)
        .value("Offline", aditof::ConnectionType::OFFLINE);

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
        .def_readwrite("bitCount", &aditof::CameraDetails::bitCount)
        .def_readwrite("uBootVersion", &aditof::CameraDetails::uBootVersion)
        .def_readwrite("kernelVersion", &aditof::CameraDetails::kernelVersion)
        .def_readwrite("sdCardImageVersion",
                       &aditof::CameraDetails::sdCardImageVersion)
        .def_readwrite("serialNumber", &aditof::CameraDetails::serialNumber);

    // Sensors declarations

    py::class_<aditof::SensorDetails>(m, "SensorDetails")
        .def(py::init<>())
        .def_readwrite("id", &aditof::SensorDetails::id)
        .def_readwrite("connectionType",
                       &aditof::SensorDetails::connectionType);

    py::class_<aditof::DriverConfiguration>(m, "DriverConfiguration")
        .def(py::init<>())
        .def_readwrite("depthBits", &aditof::DriverConfiguration::depthBits)
        .def_readwrite("abBits", &aditof::DriverConfiguration::abBits)
        .def_readwrite("confBits", &aditof::DriverConfiguration::confBits)
        .def_readwrite("pixelFormat", &aditof::DriverConfiguration::pixelFormat)
        .def_readwrite("driverWidth", &aditof::DriverConfiguration::driverWidth)
        .def_readwrite("driverHeigth",
                       &aditof::DriverConfiguration::driverHeigth)
        .def_readwrite("pixelFromatIndex",
                       &aditof::DriverConfiguration::pixelFormatIndex);

    py::class_<aditof::DepthSensorModeDetails>(m, "DepthSensorModeDetails")
        .def(py::init<>())
        .def_readwrite("numberOfPhases",
                       &aditof::DepthSensorModeDetails::numberOfPhases)
        .def_readwrite("frameContent",
                       &aditof::DepthSensorModeDetails::frameContent)
        .def_readwrite("modeNumber",
                       &aditof::DepthSensorModeDetails::modeNumber)
        .def_readwrite("pixelFormatIndex",
                       &aditof::DepthSensorModeDetails::pixelFormatIndex)
        .def_readwrite("frameWidthInBytes",
                       &aditof::DepthSensorModeDetails::frameWidthInBytes)
        .def_readwrite("frameHeightInBytes",
                       &aditof::DepthSensorModeDetails::frameHeightInBytes)
        .def_readwrite("baseResolutionWidth",
                       &aditof::DepthSensorModeDetails::baseResolutionWidth)
        .def_readwrite("baseResolutionHeight",
                       &aditof::DepthSensorModeDetails::baseResolutionHeight)
        .def_readwrite("metadataSize",
                       &aditof::DepthSensorModeDetails::metadataSize)
        .def_readwrite("isPCM", &aditof::DepthSensorModeDetails::isPCM)
        .def_readwrite("driverConfiguration",
                       &aditof::DepthSensorModeDetails::driverConfiguration);

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
            } else {
                format = "f"; // float for MP confidence frames
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
        .def(
            "getAvailableControls",
            [](const aditof::Camera &camera, py::list modes) {
                std::vector<std::string> modeList;
                aditof::Status status = camera.getAvailableControls(modeList);

                for (const auto &mode : modeList)
                    modes.append(mode);

                return status;
            },
            py::arg("availableModes"))
        .def("setMode", &aditof::Camera::setMode, py::arg("mode"))
        .def(
            "getAvailableModes",
            [](const aditof::Camera &camera, py::list modes) {
                std::vector<std::uint8_t> modeList;
                aditof::Status status = camera.getAvailableModes(modeList);

                for (const auto &mode : modeList)
                    modes.append(mode);

                return status;
            },
            py::arg("availableModes"))
        .def("getFrameProcessParams",
             [](aditof::Camera &camera) {
                 std::map<std::string, std::string> cppParams;
                 aditof::Status status =
                     camera.getFrameProcessParams(cppParams);

                 py::dict pyParams;
                 for (const auto &pair : cppParams) {
                     pyParams[py::str(pair.first)] = py::str(pair.second);
                 }

                 return std::make_pair(status, pyParams);
             })
        .def(
            "setFrameProcessParams",
            [](aditof::Camera &camera, py::dict params) {
                std::map<std::string, std::string> cppParams;

                for (std::pair<py::handle, py::handle> item : params) {
                    auto key = item.first.cast<std::string>();
                    auto value = item.second.cast<std::string>();
                    cppParams[key] = value;
                }

                return camera.setFrameProcessParams(cppParams);
            },
            py::arg("params"))
        .def("requestFrame", &aditof::Camera::requestFrame, py::arg("frame"))
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
        .def("enableXYZframe", &aditof::Camera::enableXYZframe,
             py::arg("enable"))
        .def("saveModuleCFG", &aditof::Camera::saveModuleCFG,
             py::arg("filepath"))
        .def("saveModuleCCB", &aditof::Camera::saveModuleCCB,
             py::arg("filepath"))
        .def("enableDepthCompute", &aditof::Camera::enableDepthCompute,
             py::arg("enable"))
        .def("adsd3500UpdateFirmware", &aditof::Camera::adsd3500UpdateFirmware,
             py::arg("filePath"))
        .def("saveDepthParamsToJsonFile",
             &aditof::Camera::saveDepthParamsToJsonFile,
             py::arg("savePathFile"))
        .def("loadDepthParamsFromJsonFile",
             &aditof::Camera::loadDepthParamsFromJsonFile,
             py::arg("loadPathFile"), py::arg("mode"))
        .def("setSensorConfiguration", &aditof::Camera::setSensorConfiguration,
             py::arg("sensorConf"))
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
        .def("adsd3500GetSensorTemperature",
             [](aditof::Camera &camera) {
                 uint16_t tmpValue;
                 aditof::Status status =
                     camera.adsd3500GetSensorTemperature(tmpValue);
                 return std::make_pair(status, tmpValue);
             })
        .def("adsd3500GetLaserTemperature",
             [](aditof::Camera &camera) {
                 uint16_t tmpValue;
                 aditof::Status status =
                     camera.adsd3500GetLaserTemperature(tmpValue);
                 return std::make_pair(status, tmpValue);
             })
        .def(
            "adsd3500GetFirmwareVersion",
            [](aditof::Camera &camera, std::string fwVersion,
               std::string fwHash) {
                aditof::Status status =
                    camera.adsd3500GetFirmwareVersion(fwVersion, fwHash);
                return std::make_tuple(status, fwVersion, fwHash);
            },
            py::arg("fwVersion"), py::arg("fwHash"))
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
        .def("adsd3500SetEnableMetadatainAB",
             &aditof::Camera::adsd3500SetEnableMetadatainAB, py::arg("value"))
        .def("adsd3500DisableCCBM", &aditof::Camera::adsd3500DisableCCBM,
             py::arg("value"))
        .def("adsd3500IsCCBMsupported",
             [](aditof::Camera &camera) {
                 bool supported;
                 aditof::Status status =
                     camera.adsd3500IsCCBMsupported(supported);
                 return std::make_pair(status, supported);
             })
        .def("adsd3500ResetIniParamsForMode",
             &aditof::Camera::adsd3500ResetIniParamsForMode, py::arg("value"))
        .def("adsd3500GetEnableMetadatainAB",
             [](aditof::Camera &camera) {
                 uint16_t value;
                 aditof::Status status =
                     camera.adsd3500GetEnableMetadatainAB(value);
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
             })
        .def(
            "adsd3500GetStatus",
            [](aditof::Camera &camera, int chipStatus, int imagerStatus) {
                aditof::Status status =
                    camera.adsd3500GetStatus(chipStatus, imagerStatus);
                return std::make_tuple(status, chipStatus, imagerStatus);
            },
            py::arg("chipStatus"), py::arg("imagerStatus"))
        .def("adsd3500setEnableDynamicModeSwitching",
             &aditof::Camera::adsd3500setEnableDynamicModeSwitching,
             py::arg("enable"))
        .def("adsds3500setDynamicModeSwitchingSequence",
             &aditof::Camera::adsds3500setDynamicModeSwitchingSequence,
             py::arg("sequence"))
        .def(
            "readSerialNumber",
            [](aditof::Camera &camera, std::string serialNumber,
               bool useCacheValue) {
                aditof::Status status =
                    camera.readSerialNumber(serialNumber, useCacheValue);
                return std::make_pair(status, serialNumber);
            },
            py::arg("serialNumber"), py::arg("useCacheValue"));

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
            py::arg("dataType"))
        .def("getMetadataStruct", [](aditof::Frame &frame) {
            aditof::Metadata metadata;
            aditof::Status status = frame.getMetadataStruct(metadata);
            return std::make_pair(status, metadata);
        });

    // DepthSensorInterface
    py::class_<aditof::DepthSensorInterface,
               std::shared_ptr<aditof::DepthSensorInterface>>(
        m, "DepthSensorInterface")
        .def("open", &aditof::DepthSensorInterface::open)
        .def("start", &aditof::DepthSensorInterface::start)
        .def("stop", &aditof::DepthSensorInterface::stop)
        .def(
            "getAvailableModes",
            [](aditof::DepthSensorInterface &device, py::list modes) {
                std::vector<std::uint8_t> modeList;
                aditof::Status status = device.getAvailableModes(modeList);

                for (const auto &mode : modeList)
                    modes.append(mode);

                return status;
            },
            py::arg("modes"))
        .def("getModeDetails", &aditof::DepthSensorInterface::getModeDetails,
             py::arg("mode"), py::arg("details"))
        .def(
            "setMode",
            [](aditof::DepthSensorInterface &device,
               const aditof::DepthSensorModeDetails &mode) {
                return device.setMode(mode);
            },
            py::arg("mode"))
        .def(
            "setMode",
            [](aditof::DepthSensorInterface &device, const uint8_t &mode) {
                return device.setMode(mode);
            },
            py::arg("mode"))
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
            "getAvailableControls",
            [](const aditof::DepthSensorInterface &device, py::list controls) {
                std::vector<std::string> controlsList;
                aditof::Status status =
                    device.getAvailableControls(controlsList);

                for (const auto &control : controlsList)
                    controls.append(control);

                return status;
            },
            py::arg("controls"))
        .def(
            "adsd3500_read_cmd",
            [](aditof::DepthSensorInterface &device, uint16_t cmd,
               py::array_t<uint16_t> data, unsigned int usDelay) {
                py::buffer_info dataBuffInfo = data.request();
                uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);
                aditof::Status status =
                    device.adsd3500_read_cmd(cmd, dataPtr, usDelay);
                return std::make_pair(status, dataPtr);
            },
            py::arg("cmd"), py::arg("data"), py::arg("usDelay"))
        .def(
            "adsd3500_write_cmd",
            [](aditof::DepthSensorInterface &device, uint16_t cmd,
               uint16_t data) {
                aditof::Status status = device.adsd3500_write_cmd(cmd, data);
                return status;
            },
            py::arg("cmd"), py::arg("data"))
        .def(
            "adsd3500_read_payload_cmd",
            [](aditof::DepthSensorInterface &device, uint32_t cmd,
               py::array_t<uint8_t> readback_data, uint16_t payload_len) {
                py::buffer_info readbackBuffInfo = readback_data.request();
                uint8_t *readback_dataPtr =
                    static_cast<uint8_t *>(readbackBuffInfo.ptr);
                aditof::Status status = device.adsd3500_read_payload_cmd(
                    cmd, readback_dataPtr, payload_len);
                return std::make_pair(status, readback_dataPtr);
            },
            py::arg("cmd"), py::arg("readback_data"), py::arg("payload_len"))
        .def(
            "adsd3500_read_payload",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint8_t> payload, uint16_t payload_len) {
                py::buffer_info payloadBuffInfo = payload.request();
                uint8_t *payloadPtr =
                    static_cast<uint8_t *>(payloadBuffInfo.ptr);
                aditof::Status status =
                    device.adsd3500_read_payload(payloadPtr, payload_len);
                return std::make_pair(status, payloadPtr);
            },
            py::arg("payload"), py::arg("payload_len"))
        .def(
            "adsd3500_write_payload_cmd",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint8_t> payload, uint32_t cmd,
               uint16_t payload_len) {
                py::buffer_info payloadBuffInfo = payload.request();
                uint8_t *payloadPtr =
                    static_cast<uint8_t *>(payloadBuffInfo.ptr);
                aditof::Status status = device.adsd3500_write_payload_cmd(
                    cmd, payloadPtr, payload_len);
                return status;
            },
            py::arg("cmd"), py::arg("payload"), py::arg("payload_len"))
        .def(
            "adsd3500_write_payload",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint8_t> payload, uint16_t payload_len) {
                py::buffer_info payloadBuffInfo = payload.request();
                uint8_t *payloadPtr =
                    static_cast<uint8_t *>(payloadBuffInfo.ptr);
                aditof::Status status =
                    device.adsd3500_write_payload(payloadPtr, payload_len);
                return status;
            },
            py::arg("payload"), py::arg("payload_len"))
        .def("adsd3500_reset",
             [](aditof::DepthSensorInterface &device) {
                 aditof::Status status = device.adsd3500_reset();
                 return status;
             })
        .def(
            "adsd3500_register_interrupt_callback",
            [](aditof::DepthSensorInterface &device,
               aditof::SensorInterruptCallback &cb) {
                return device.adsd3500_register_interrupt_callback(cb);
            },
            py::arg("cb"))
        .def(
            "adsd3500_unregister_interrupt_callback",
            [](aditof::DepthSensorInterface &device,
               aditof::SensorInterruptCallback &cb) {
                return device.adsd3500_unregister_interrupt_callback(cb);
            },
            py::arg("cb"))
        .def("setControl", &aditof::DepthSensorInterface::setControl,
             py::arg("control"), py::arg("value"))
        .def("getControl", &aditof::DepthSensorInterface::getControl,
             py::arg("control"), py::arg("value"))
        .def("getDetails", &aditof::DepthSensorInterface::getDetails,
             py::arg("details"))
        .def(
            "getHandle",
            [](aditof::DepthSensorInterface &device, void *handle) {
                aditof::Status status =
                    device.getHandle(static_cast<void **>(handle));
                return std::make_pair(status, handle);
            },
            py::arg("handle"))
        .def("getName", &aditof::DepthSensorInterface::getName, py::arg("name"))
        .def("setHostConnectionType",
             &aditof::DepthSensorInterface::setHostConnectionType,
             py::arg("connectionType"))
        .def(
            "initTargetDepthCompute",
            [](aditof::DepthSensorInterface &device,
               py::array_t<uint8_t> iniFile, uint16_t iniFileLength,
               py::array_t<uint8_t> calData, uint16_t calDataLength) {
                py::buffer_info iniFileBuffInfo = iniFile.request();
                uint8_t *iniFilePtr =
                    static_cast<uint8_t *>(iniFileBuffInfo.ptr);
                py::buffer_info calDataBuffInfo = calData.request();
                uint8_t *calDataPtr =
                    static_cast<uint8_t *>(calDataBuffInfo.ptr);
                aditof::Status status = device.initTargetDepthCompute(
                    iniFilePtr, iniFileLength, calDataPtr, calDataLength);
                return std::make_tuple(status, iniFilePtr, calDataPtr);
            },
            py::arg("iniFile"), py::arg("iniFileLength"), py::arg("calData"),
            py::arg("calDataLength"));

    // FrameHandler
    py::class_<aditof::FrameHandler>(m, "FrameHandler")
        .def(py::init<>())
        .def("setOutputFilePath", &aditof::FrameHandler::setOutputFilePath,
             py::arg("filePath"))
        .def("setInputFileName", &aditof::FrameHandler::setInputFileName,
             py::arg("fullFileName"))
        .def("saveFrameToFile", &aditof::FrameHandler::saveFrameToFile,
             py::arg("frame"), py::arg("fileName") = "")
        .def("saveFrameToFileMultithread",
             &aditof::FrameHandler::saveFrameToFileMultithread,
             py::arg("frame"), py::arg("fileName") = "")
        .def("readNextFrame", &aditof::FrameHandler::readNextFrame,
             py::arg("frame"), py::arg("fullFileName"),
             py::return_value_policy::reference_internal)
        .def("setCustomFormat", &aditof::FrameHandler::setCustomFormat,
             py::arg("format"))
        .def("storeFramesToSingleFile",
             &aditof::FrameHandler::storeFramesToSingleFile, py::arg("enable"))
        .def("setFrameContent", &aditof::FrameHandler::setFrameContent,
             py::arg("frameContent"));

    //SDK version
    m.def("getApiVersion", &aditof::getApiVersion);
    m.def("getBranchVersion", &aditof::getBranchVersion);
    m.def("getCommitVersion", &aditof::getCommitVersion);
}
