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
     * @brief Get the number of modes supported
     * @return int
     */
    unsigned int getNumModes();

  private:
    static modeInfo g_modeInfoData[]; //static array of all modes supported
    static ModeInfo *m_instance;      //single instance
    ModeInfo() {} // private so that it can not be called, always access through getInstance()
    ModeInfo(const ModeInfo &);            // copy constructor is private
    ModeInfo &operator=(const ModeInfo &); // assignment operator is private
};

#endif // MODE_INFO
