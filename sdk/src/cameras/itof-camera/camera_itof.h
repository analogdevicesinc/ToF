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
#ifndef CAMERA_ITOF_H
#define CAMERA_ITOF_H

#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/storage_interface.h>
#include <aditof/temperature_sensor_interface.h>
#include <map>
#include <unordered_map>
// #include "calibration.h"
 #include "tofi/tofi_compute.h"
 #include "tofi/tofi_config.h"
// #include "aditof_internal.h"
// #include "tofi_utils.h"
#include "mode_info.h"

/* Camera controls
 *
 * initialization_config
 *   Description:     The JSON configuration files which should be specific to the given module.
 *                    Can be omitted if 'loadModuleData' control is executed. This needs to be set
 *                    before calling initialize().
 *   Accepted values: A file name, including extension.
 *
 * powerUp
 *   Description:     PowerUp the camera, it performs power monitoring on different power registers.
 *                    This is required before performing any operation on the camera.
 *   Accepted values: A string with this exact value: 'call'
 *
 * powerDown
 *   Description:     PowerDown the camera, this is required for shutting down the camera module.
 *   Accepted values: A string with this exact value: 'call'.
 *
 * syncMode
 *   Description:     Setting frame sync, either internal or external (default is internal)
 *   Accepted values: A string containing two integer values. E.g. '1, 2'
 *                    First integer corresponds to mode. Set to 0 for internal and 2 for
 *                    external. Second integer is level. Set to TDB!!!
 * loadModuleData
 *   Description:     Read camera module memory and initialize camera with loaded data.
 *                    Must be called after initialize() and powerUp(). Calibration or Firmware
 *                    data are NOT loaded if already defined by initialize() config file.
 *   Accepted values: A string with this exact value: 'call'
 *
 * enableDepthCompute
 *   Description:     Enable or disable the depth processing on the frames received from the sensor
 *                    Must be called after getFrame() where the depth processing happens.
 *   Accepted values: One of the following strings: 'on' or 'off'
 */

class CameraItof : public aditof::Camera {
  public:
    CameraItof(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
               std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
               std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
                   &tSensors);
    ~CameraItof();

  public: // implements Camera
    aditof::Status initialize() override;
    aditof::Status start() override;
    aditof::Status stop() override;
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename) override;
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const override;
    aditof::Status setFrameType(const std::string &frameType) override;
    aditof::Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const override;
    aditof::Status requestFrame(aditof::Frame *frame,
                                aditof::FrameUpdateCallback cb) override;
    aditof::Status getDetails(aditof::CameraDetails &details) const override;
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    aditof::Status setControl(const std::string &control,
                              const std::string &value) override;
    aditof::Status getControl(const std::string &control,
                              std::string &value) const override;
    std::shared_ptr<aditof::DepthSensorInterface> getSensor() override;
    aditof::Status
    getEeproms(std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms)
        override;
    aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &sensors) override;

    // TO DO - The methods bellow need to be converted somehow to be covered by setControl()
    // in order to not go beyond Camera API

    /**
     * @brief Write data to  camera module memory
     *
     * This will attempt to write to module Flash/Eeprom memory and will
     * permanently overwrite any existing data.
     *
     * @param[in] ccbFileName - calibration data to write to module memory
     * @param[in] cfgFileName - configuration data to write to module memory
     * @return Status
     * @see Status
     */
    //aditof::Status writeModuleEeprom(const std::string &ccbFileName, const std::string &cfgFileName);

  private:

    /**
     * @brief Default ADI module flash memory is Macronix MX25U6435F.
     * Override by defining a MODULE_EEPROM_TYPE name in configuration JSON file.
     */
    std::string m_eepromDeviceName = "MX25U6435F";

    /**
     * @brief Initializes the Depth compute libraries
     * @return aditof::Status
     * @see aditof::Status
     * @note TOF SDK uses the dlls generated by the depth compute libraries, see ../lib/windows folder
     */
    aditof::Status initComputeLibrary(void);

    /**
     * @brief Frees all the resources created by depth compute/config library
     * @return aditof::Status
     * @see aditof::Status
     * @note TOF SDK uses the dlls generated by the depth compute libraries, see ../lib/windows folder
     */
    aditof::Status freeComputeLibrary(void);

    /**
     * @brief Opens the CCB file passed in as part of Json file using initialize(), and loads the calibration blocks into member variable
     * @return std::tuple<aditof::Status, int, int, int>
     * @see aditof::Status
     * @see <a href='../config/config_default.json'>config_default.json</a>
     */
    std::tuple<aditof::Status, int, int, int> loadConfigData(void);

    /**
     * @brief Frees the calibration member variables created while loading the CCB contents
     * @return None
     */
    void freeConfigData(void);

    /**
     * @brief Tests whether SDK received valid frame feild with valid totalcaptures set for particular mode.
     * @param[in] numTotalFrames - This is received Totalcaptures value after parsing the incoming frame.
     *                             This will be verified aganist the expected Totalcaptures for a particular mode set
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status isValidFrame(const int numTotalFrames);

    /**
     * @brief Tests whether SDK received valid mode feild based on what mode user has set.
     * @param[in] hdr_mode - This is received Mode value after parsing the incoming frame.
     *                       This will be verified aganist the expected Mode value. This is make sure when the user changes the mode, there might be
     *                       still some frames stored in UVC pipe for previous mode, this check drops those frames until we get new frame with the new mode feild in the embedded header
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status isValidMode(const uint8_t hdr_mode);

    /**
     * @brief Processes the incoming frame, verifies whether the frame has all valid subcaptures, parses through each subcapture and verifies each sub-capture has valid
     *        CHIP_ID, CaptureID, Frame number. It stores the raw capture, embedded header in separate buffer passed in
     * @param[in] rawFrame - Incoming Raw frame with headers and actual capture data
     * @param[in] embed_height - rawFrame height
     * @param[in] embed_width - rawFrame width
     * @param[in/out] frameDetails - send in the frameDetails for a particular frame, frameNumber for a frame is updated
     * @param[out] captureData - Valid buffer passed in to store the capture data, later this will be used to calculate the depth.
     * @param[out] head - Valid buffer passed in to store the cheader feilds.
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status processFrame(uint8_t *rawFrame, uint16_t *captureData,
                                uint8_t *head, const uint16_t embed_height,
                                const uint16_t embed_width,
                                aditof::Frame *frame);

    /**
     * @brief Gets the information about the current mode set
     * @param[out] info - This gets filled with all the mode information (width, height, subframes, embed_width, embed_height)
     * @return aditof::Status
     * @note This function has to be called after calling setMode(), this is private function,
     * @see ModeInfo::modeInfo
     * @see aditof::Status
     */
    aditof::Status getCurrentModeInfo(ModeInfo::modeInfo &info);

    /**
     * @brief Delete any temporary files created in camera initialization
     * @return aditof::Status
     * @see m_tempFiles
     */
    aditof::Status cleanupTempFiles();

    /**
     * @brief PowerUp the camera, it performs power monitoring on different power registers.
     * This is required before performing any operation on the camera.
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status powerUp();

    /**
     * @brief PowerDown the camera, this is required for shutting down the camera module
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status powerDown();

    /**
     * @brief Setting frame sync, either internal or external (default is internal)
     * @param mode - external/internal
     * @return Status
     */
    aditof::Status setCameraSyncMode(uint8_t mode, uint8_t level);

    /**
     * @brief Read camera module memory and initialize camera with loaded data.
     *
     * Must be called after initialize() and powerUp(). Calibration or Firmware
     * data are NOT loaded if already defined by initialize() config file.
     *
     * @return Status
     * @see Status
     */
    aditof::Status loadModuleData();

    /**
     * @brief Apply calibration to the frame captured
     * Based on the platform the camera is connected to Windows/Linux/Mac corresponding implementation of applyCalibrationToFrame is done.
     * @param[in] frame - Frame captured
     * @param[in] mode - Mode the camera is set work in
     * @return aditof::Status
     * @see aditof::Status
     */
    aditof::Status applyCalibrationToFrame(uint16_t *frame, const unsigned int mode);

  private:
    using noArgCallable = std::function<aditof::Status()>;

    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::shared_ptr<aditof::StorageInterface> m_eeprom;
    std::shared_ptr<aditof::TemperatureSensorInterface> m_tempSensor;
    std::unordered_map<std::string, std::string> m_controls;
    std::map<std::string, noArgCallable> m_noArgCallables;

    bool m_devStarted;
    bool m_eepromInitialized;
    bool m_tempSensorInitialized;
    // Calibration m_calibration;

    uint8_t *m_calData = NULL;
    uint8_t *m_depthINIData = NULL;
    uint8_t *m_jconfigData = NULL;
    TofiConfig *m_tofi_config = NULL;
    TofiComputeContext *m_tofi_compute_context = NULL;

    bool m_CameraProgrammed = false;
    std::string m_sensorFirmwareFile;
    std::string m_ccb_calibrationFile;
    std::string m_ini_depth;
    uint16_t m_modechange_framedrop_count = 0;
    std::vector<std::string> m_tempFiles;
    std::vector<aditof::DepthSensorFrameType> m_availableSensorFrameTypes;
};

#endif // CAMERA_ITOF_H
