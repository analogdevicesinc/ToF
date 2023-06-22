/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef MODE_INFO
#define MODE_INFO

#include <aditof/status_definitions.h>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @class ModeInfo
 * @brief Singleton definition of Mode details (width, height, total captures, embedded_width, embedded_height, passive_ir, mode_name)
 * @note If low level USB objects or camera objects needs to get information of particular mode, they can use the single instance to get mode information
 */
class ModeInfo {

  public:
    /**
    * @struct modeInfo
    * @brief Maintains the information of all supported modes
    */
    typedef struct {
        uint8_t mode;
        uint16_t width;
        uint16_t height;
        uint8_t subframes;
        uint16_t embed_width;
        uint16_t embed_height;
        uint8_t passive_ir;
        std::string mode_name;
    } modeInfo;

    /**
     * @brief Get the single instance of the modeinfo
     * @return ModeInfo*
     * @see ModeInfo
     * @note If low level USB objects or camera objects needs to get information of particular mode, they can use the single instance to get mode information
     */
    static ModeInfo *getInstance();

    /**
     * @brief Get the modeinformation based on mode
     * param[in] mode - Mode for which the information is needed
     * @return modeInfo*
     * @see modeInfo
     */
    modeInfo getModeInfo(unsigned int mode);

    /**
     * @brief Get the modeinformation based on mode
     * param[in] mode - Mode for which the information is needed
     * @return modeInfo*
     * @see modeInfo
     */
    modeInfo getModeInfo(const std::string &mode);

    /**
     * @brief Set imager type. Set mode version.
     * param[in] type - 1 ADSD3100, 2 ADSD3030
     * param[in] version - 0 Old table, 1 new adsd3100 table, 2 new adsd3500 table
     * @return aditof::Status
     */
    aditof::Status setImagerTypeAndModeVersion(int version, int type);

    /**
     * Get the id of the mode that corresponds to the name
     * param[in] modes - the name of the mode (represented as a string)
     * param[out] convertedMode - the id of the mode (represented as a number)
     * @return aditof::Status
    */
    aditof::Status convertCameraMode(const std::string &modes,
                                     uint8_t &convertedMode);

    std::vector<std::string> GetAvailableModes() { return m_availableModes; };

  private:
    static std::vector<modeInfo> g_modeInfoData;
    static modeInfo g_newModesAdsd3100[];
    static modeInfo g_newModesAdsd3500Adsd3100[];
    static modeInfo g_newModesAdsd3500Adsd3030[];
    static modeInfo g_newMixedModesAdsd3500Adsd3100[];
    static modeInfo g_newMixedModesAdsd3500Adsd3030[];
    static ModeInfo *m_instance; //single instance
    static int g_imagerType;
    static int g_modeVersion;
    std::vector<std::string> m_availableModes;

    ModeInfo() {
    } // private so that it can not be called, always access through getInstance()
    ModeInfo(const ModeInfo &);            // copy constructor is private
    ModeInfo &operator=(const ModeInfo &); // assignment operator is private

    /**
     * Populate g_availableModes with the modes for the selected mode
     * param[in] modes - modes array
     * @return aditof::Status
    */
    aditof::Status
    populateAvailableModes(const std::vector<ModeInfo::modeInfo> modes);
};

#endif // MODE_INFO
