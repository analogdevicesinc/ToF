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
#ifndef CAMERA_H
#define CAMERA_H

#include "camera_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <functional>
#include <string>
#include <vector>

namespace aditof {

class Frame;
class DepthSensorInterface;
class StorageInterface;
class TemperatureSensorInterface;

/**
 * @class Camera
 * @brief Manipulates the underlying camera system
 */
class SDK_API Camera {
  public:
    /**
     * @brief Destructor
     */
    virtual ~Camera() = default;

    /**
     * @brief Initialize the camera. This is required before performing any
     * operation on the camera.
     * @return Status
     */
    virtual Status initialize() = 0;

    /**
     * @brief Start the camera. This starts the streaming of data from the
     * camera.
     * @return Status
     */
    virtual Status start() = 0;

    /**
     * @brief Stop the camera. This makes the camera to stop streaming.
     * @return Status
     */
    virtual Status stop() = 0;

    /**
     * @brief Puts the camera into the given mode.
     * @param mode - The mode of the camera
     * @param modeFilename - When there is a need to use a custom mode
     * then mode parameter needs to be set to 'custom' and a firmware
     * file needs to be provided.
     * @return Status
     */
    virtual Status setMode(const std::string &mode,
                           const std::string &modeFilename = {}) = 0;

    /**
     * @brief Returns all the modes that are supported by the camera
     * @param[out] availableModes
     * @return Status
     */
    virtual Status
    getAvailableModes(std::vector<std::string> &availableModes) const = 0;

    /**
     * @brief Set the camera frame type to the given type
     * @param frameType - The frame type of the camera
     * @return Status
     */
    virtual Status setFrameType(const std::string &frameType) = 0;

    /**
     * @brief Returns all the frame types that are supported by the camera
     * @param[out] availableFrameTypes
     * @return Status
     */
    virtual Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const = 0;

    /**
     * @brief Returns the name of the frame type for the given ID
     * @param id - the ID of the frame type
     * @param[out] name - will be set with the name of the frame type in case of success
     * @return Status
     */
    virtual Status getFrameTypeNameFromId(int id, std::string &name) const = 0;

    /**
     * @brief Captures data from the camera and assigns it to the given frame.
     * If cb parameter is not given this operation will be blocking. If a
     * callback is provided this operation will be unblocking and once the data
     * for the frame is ready, an internal thread will call the specified
     * callback.
     * @param frame - The frame to which the camera data should be assign
     * @param cb - Callback to be called when frame is updated
     * @return Status
     */
    virtual Status requestFrame(Frame *frame,
                                FrameUpdateCallback cb = nullptr) = 0;

    /**
     * @brief Gets the current details of the camera
     * @param[out] details
     * @return Status
     */
    virtual Status getDetails(CameraDetails &details) const = 0;

    /**
     * @brief Gets the camera's list of controls
     * @param[out] controls
     * @return Status
     */
    virtual Status
    getAvailableControls(std::vector<std::string> &controls) const = 0;

    /**
     * @brief Sets a specific camera control
     * @param[in] control - Control name
	 * @param[in] value - Control value
     * @return Status
     */
    virtual Status setControl(const std::string &control,
                              const std::string &value) = 0;

    /**
     * @brief Gets the value of a specific camera control
     * @param[in] control - Control name
	 * @param[out] value - Control value
     * @return Status
     */
    virtual Status getControl(const std::string &control,
                              std::string &value) const = 0;

    /**
     * @brief Gets the sensor of the camera. This gives direct access
     * to low level configuration of the camera sensor.
     * @return std::shared_ptr<DepthSensorInterface>
     */
    virtual std::shared_ptr<DepthSensorInterface> getSensor() = 0;

    /**
     * @brief Gets the eeprom(s) used internally by the camera. This gives
     * direct access to the eeprom(s) of the camera.
     * @param[out] eeproms - List of internal eeproms
     * @return Status
     */
    virtual Status
    getEeproms(std::vector<std::shared_ptr<StorageInterface>> &eeproms) = 0;

    /**
     * @brief Gets the temperature sensors used internally by the camera.
     * This gives direct access to the temperature sensor(s) of the camera.
     * @param[out] sensors - List of internal temperature sensors
     * @return Status
     */
    virtual Status getTemperatureSensors(
        std::vector<std::shared_ptr<TemperatureSensorInterface>> &sensors) = 0;

    /**
     * @brief Enables or disables FSYNC toggle for ADSD3500
     * @param[in] mode - 2 = Fsync pin set as HiZ ; 1 = Toggle at user specified framerate ; 0 = Toggle controlled via adsd3500_toggle_fsync ; 
     * @return Status
     */
    virtual Status adsd3500_set_toggle_mode(int mode) = 0;

    /**
     * @brief Toggles ADSD3500 FSYNC once if automated FSYNC is disabled
     * @return Status
     */
    virtual Status adsd3500_toggle_fsync() = 0;

    /**
     * @brief Set the AB invalidation threshold
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetABinvalidationThreshold(int threshold) = 0;

    /**
     * @brief Get the AB invalidation threshold
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetABinvalidationThreshold(int &threshold) = 0;

    /**
     * @brief Set the confidence threshold
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetConfidenceThreshold(int threshold) = 0;

    /**
     * @brief Get the confidence threshold
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetConfidenceThreshold(int &threshold) = 0;

    /**
     * @brief Enable/disable the JBLF filter
     * @param[in] enable
     * @return Status
     */
    virtual Status adsd3500SetJBLFfilterEnableState(bool enable) = 0;

    /**
     * @brief Get the JBLF enabled state
     * @param[out] enabled
     * @return Status
     */
    virtual Status adsd3500GetJBLFfilterEnableState(bool &enabled) = 0;

    /**
     * @brief Set the JBLF filter size
     * @param[in] size - Supported sizes are: 3, 5, 7
     * @return Status
     */
    virtual Status adsd3500SetJBLFfilterSize(int size) = 0;

    /**
     * @brief Get the JBLF filter size
     * @param[out] size
     * @return Status
     */
    virtual Status adsd3500GetJBLFfilterSize(int &size) = 0;

    /**
     * @brief Set the radial threshold min
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetRadialThresholdMin(int threshold) = 0;

    /**
     * @brief Get the radial threshold min
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetRadialThresholdMin(int &threshold) = 0;

    /**
     * @brief Set the radial threshold max
     * @param[in] threshold
     * @return Status
     */
    virtual Status adsd3500SetRadialThresholdMax(int threshold) = 0;

    /**
     * @brief Get the radial threshold max
     * @param[out] threshold
     * @return Status
     */
    virtual Status adsd3500GetRadialThresholdMax(int &threshold) = 0;

    /**
     * @brief Get the sensor temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    virtual Status adsd3500GetSensorTemperature(uint16_t &tmpValue) = 0;

    /**
     * @brief Get the laser temperature
     * @param[out] tmpValue - Values in Celsius degree
     * @return Status
     */
    virtual Status adsd3500GetLaserTemperature(uint16_t &tmpValue) = 0;

    /**
     * Get the ASDSD3500 firmware version from the ADSD3500
     * @param[out] fwVersion - the ADSD3500 firmware version
     * @param[out] fwHash - the ADSD3500 firmware git commit hash
    */
    virtual Status adsd3500_get_firmware_Version(std::string &fwVersion,
                                                 std::string &fwHash) = 0;
};

} // namespace aditof

#endif // CAMERA_H
