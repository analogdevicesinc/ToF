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
#include <glog/logging.h>
#include <ostream>
#include <string>
#include <vector>

const std::vector<std::string> g_availableModes = {"pcmmp", "srqmp", "lrqmp",
                                                   "srmp", "lrmp"};

aditof::Status convertCameraMode(const std::string &modes,
                                 uint8_t &convertedMode);

/**
 * @class ModeInfo
 * @brief Singleton definition of Mode details (width, height, total captures, embedded_width, embedded_height)
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
     * @brief Get the number of modes supported
     * @return int
     */
    unsigned int getNumModes();

    /**
     * @brief Set mode version
     * param[in] version - 0 Old table, 1 new adsd3100 table, 2 new adsd3500 table
     * @return aditof::Status
     */
    aditof::Status setModeVersion(int version);

    /**
     * @brief Set mode version
     * @return 1 if new ModeInfo table/ 0 for old table
     */
    int getModeVersion();

  private:
    static modeInfo *g_modeInfoData;      //static array of all modes supported
    static modeInfo g_oldModes[];         //static array of old modes supported
    static modeInfo g_newModesAdsd3100[]; //static array of new modes supported
    static modeInfo g_newModesAdsd3500[]; //static array of new modes supported
    static modeInfo g_adsd3030Modes[];    //static array of adsd3030 modes
    static ModeInfo *m_instance;          //single instance

    ModeInfo() {
    } // private so that it can not be called, always access through getInstance()
    ModeInfo(const ModeInfo &);            // copy constructor is private
    ModeInfo &operator=(const ModeInfo &); // assignment operator is private
};

#endif // MODE_INFO
