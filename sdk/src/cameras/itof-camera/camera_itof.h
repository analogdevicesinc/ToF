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

#include "mode_info.h"
#include "tofi/tofi_compute.h"
#include "tofi/tofi_config.h"
#include "tofi/tofi_util.h"
#include <aditof/adsd_errs.h>
#include <aditof/camera.h>
#include <aditof/depth_sensor_interface.h>
#include <map>
#include <unordered_map>

class CameraItof : public aditof::Camera {
  public:
    CameraItof(std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
               const std::string &ubootVersion,
               const std::string &kernelVersion,
               const std::string &sdCardImageVersion);
    ~CameraItof();

  public: // implements Camera
    aditof::Status initialize(const std::string &configFilepath = {}) override;
    aditof::Status start() override;
    aditof::Status stop() override;
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename) override;
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const override;
    aditof::Status setFrameType(const std::string &frameType) override;
    aditof::Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const override;
    aditof::Status getFrameTypeNameFromId(int id,
                                          std::string &name) const override;
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
    aditof::Status enableXYZframe(bool enable) override;
    aditof::Status saveModuleCFG(const std::string &filepath) const override;
    aditof::Status saveModuleCCB(const std::string &filepath) override;
    aditof::Status enableDepthCompute(bool enable) override;
    aditof::Status adsd3500UpdateFirmware(const std::string &filePath) override;
    aditof::Status adsd3500SetToggleMode(int mode) override;
    aditof::Status adsd3500ToggleFsync() override;
    aditof::Status adsd3500SetABinvalidationThreshold(int threshold) override;
    aditof::Status adsd3500GetABinvalidationThreshold(int &threshold) override;
    aditof::Status adsd3500SetConfidenceThreshold(int threshold) override;
    aditof::Status adsd3500GetConfidenceThreshold(int &threshold) override;
    aditof::Status adsd3500SetJBLFfilterEnableState(bool enable) override;
    aditof::Status adsd3500GetJBLFfilterEnableState(bool &enabled) override;
    aditof::Status adsd3500SetJBLFfilterSize(int size) override;
    aditof::Status adsd3500GetJBLFfilterSize(int &size) override;
    aditof::Status adsd3500SetRadialThresholdMin(int threshold) override;
    aditof::Status adsd3500GetRadialThresholdMin(int &threshold) override;
    aditof::Status adsd3500SetRadialThresholdMax(int threshold) override;
    aditof::Status adsd3500GetRadialThresholdMax(int &threshold) override;
    aditof::Status adsd3500GetSensorTemperature(uint16_t &tmpValue) override;
    aditof::Status adsd3500GetLaserTemperature(uint16_t &tmpValue) override;
    aditof::Status adsd3500GetFirmwareVersion(std::string &fwVersion,
                                              std::string &fwHash) override;
    aditof::Status adsd3500SetMIPIOutputSpeed(uint16_t speed) override;
    aditof::Status adsd3500GetMIPIOutputSpeed(uint16_t &speed) override;
    aditof::Status adsd3500GetImagerErrorCode(uint16_t &errcode) override;
    aditof::Status adsd3500SetVCSELDelay(uint16_t delay) override;
    aditof::Status adsd3500GetVCSELDelay(uint16_t &delay) override;
    aditof::Status adsd3500SetJBLFMaxEdgeThreshold(uint16_t threshold) override;
    aditof::Status adsd3500SetJBLFABThreshold(uint16_t threshold) override;
    aditof::Status adsd3500SetJBLFGaussianSigma(uint16_t value) override;
    aditof::Status adsd3500GetJBLFGaussianSigma(uint16_t &value) override;
    aditof::Status adsd3500SetJBLFExponentialTerm(uint16_t value) override;
    aditof::Status adsd3500GetJBLFExponentialTerm(uint16_t &value) override;
    aditof::Status adsd3500GetFrameRate(uint16_t &fps) override;
    aditof::Status adsd3500SetFrameRate(uint16_t fps) override;
    aditof::Status adsd3500SetEnableEdgeConfidence(uint16_t value) override;
    aditof::Status
    adsd3500GetTemperatureCompensationStatus(uint16_t &value) override;
    aditof::Status adsd3500SetEnablePhaseInvalidation(uint16_t value) override;
    aditof::Status
    adsd3500SetEnableTemperatureCompensation(uint16_t value) override;
    aditof::Status adsd3500SetEnableEmbeddedHeaderinAB(uint16_t value) override;
    aditof::Status
    adsd3500GetEnableEmbeddedHeaderinAB(uint16_t &value) override;
    aditof::Status adsd3500SetGenericTemplate(uint16_t reg,
                                              uint16_t value) override;
    aditof::Status adsd3500GetGenericTemplate(uint16_t reg,
                                              uint16_t &value) override;
    aditof::Status adsd3500GetStatus(int &chipStatus,
                                     int &imagerStatus) override;

    aditof::Status readSerialNumber(std::string &serialNumber,
                                    bool useCacheValue = false) override;

  private:
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
     * @return aditof::Status
     * @see aditof::Status
     * @see <a href='../config/config_default.json'>config_default.json</a>
     */
    aditof::Status loadConfigData(void);

    /**
     * @brief Frees the calibration member variables created while loading the CCB contents
     * @return None
     */
    void freeConfigData(void);

    /**
     * @brief Gets the information about the current mode set
     * @param[out] info - This gets filled with all the mode information (width, height, subframes, embed_width, embed_height)
     * @return aditof::Status
     * @note This function has to be called after calling setMode(), this is private function,
     * @see ModeInfo::modeInfo
     * @see aditof::Status
     */
    aditof::Status getCurrentModeInfo(ModeInfo::modeInfo &info);

    // Methods available only when Adsd3500 is detected as part of the entire setup

    /**
     * @brief Read the ccb from adsd3500 memory and store it in m_tempFiles.ccbFile
     *
     * @return Status
     * @see Status
     */
    aditof::Status readAdsd3500CCB();

    /**
     * Configure the sensor with various settings that affect the frame type.
     */
    void configureSensorFrameType();

    /**
     * Reads the content of json file and populates the parameters
     */
    aditof::Status parseJsonFileContent();

    /**
     * Get key-value pairs from ini file
     * @param[in] iniFileName - the name of the ini file to be opened and parsed
     * @param[out] iniKeyValPairs - map with parameter names and their values extracted from ini file
     * @return Status
     * @see Status
    */
    aditof::Status
    getKeyValuePairsFromIni(const std::string &iniFileName,
                            std::map<std::string, std::string> &iniKeyValPairs);

    /**
     * Configure ADSD3500 with ini parameters
     * @param[in] iniKeyValPairs - ini parameteres to use
    */
    void setAdsd3500WithIniParams(
        const std::map<std::string, std::string> &iniKeyValPairs);

    /**
     * @brief Delete allocated tables for X, Y, Z
     */
    void cleanupXYZtables();

  private:
    using noArgCallable = std::function<aditof::Status()>;

    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::unordered_map<std::string, std::string> m_controls;
    std::map<std::string, noArgCallable> m_noArgCallables;
    aditof::ADSDErrors m_adsdErrors;

    bool m_devStarted;
    bool m_tempSensorInitialized;
    bool m_adsd3500Enabled;
    bool m_adsd3500_master;
    bool m_isOffline;

    FileData m_calData = {NULL, 0};

    FileData m_depthINIData;
    std::map<std::string, FileData> m_depthINIDataMap;
    uint8_t *m_jconfigData = NULL;
    TofiConfig *m_tofi_config = NULL;
    TofiComputeContext *m_tofi_compute_context = NULL;
    TofiXYZDealiasData m_xyz_dealias_data[MAX_N_MODES + 1];
    bool m_loadedConfigData;

    std::string m_sensorFirmwareFile;
    std::string m_ccb_calibrationFile;
    std::string m_ini_depth;
    std::map<std::string, std::string> m_ini_depth_map;
    bool m_abEnabled;
    uint8_t m_abBitsPerPixel;
    bool m_xyzEnabled;
    bool m_xyzSetViaApi;
    bool m_pcmFrame;
    std::vector<aditof::DepthSensorFrameType> m_availableSensorFrameTypes;
    std::vector<std::pair<std::string, int32_t>> m_sensor_settings;
    int16_t m_cameraFps;
    int16_t m_fsyncMode;
    int16_t m_mipiOutputSpeed;
    int16_t m_enableTempCompenstation;
    int16_t m_enableMetaDatainAB;
    int16_t m_enableEdgeConfidence;
    std::map<std::string, std::string> m_iniKeyValPairs;
    //pair between firmware version and git hash
    std::pair<std::string, std::string> m_adsd3500FwGitHash;
    int m_adsd3500FwVersionInt;
    int m_adsd3500ImagerType;
    int m_modesVersion;
    bool m_fwUpdated;
    aditof::Adsd3500Status m_adsd3500Status;
    bool m_targetFramesAreComputed;
    XYZTable m_xyzTable;
    bool m_enableDepthCompute;
    std::string m_initConfigFilePath;
    std::string m_ccbFile;
};

#endif // CAMERA_ITOF_H
