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

#ifndef HTTP_PROTOCOL_TOF
#define HTTP_PROTOCOL_TOF

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#if !defined(LWS_PLUGIN_STATIC)
#define LWS_DLL
#define LWS_INTERNAL
#include <libwebsockets.h>
#endif

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <algorithm>
#include <iostream>
#include <string>

using namespace std;
using namespace aditof;

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define CHECK_PERIOD_US 50000
#define HEADER_SIZE 0
#define MAX_MESSAGE_LEN                                                        \
    (LWS_PRE + 1042 * 1024 * 2 + HEADER_SIZE) // Maximal pixel size
#define BITS_PER_PIXEL 8

struct pss__tof {
    int number;
};

struct vhd__tof {
    const unsigned int *options;
};

struct lws *systemWsi; // Websocket pointer

//Tof related variables
aditof::System tofSystem;
std::vector<std::shared_ptr<aditof::Camera>> cameras;
shared_ptr<aditof::Camera> camera = NULL;

// Frame related variables
std::shared_ptr<aditof::Frame> frame;
uint16_t frameWidth;
uint16_t frameHeight;
uint16_t *frameData = NULL;
uint16_t *frameDataDepth;
uint16_t *frameDataAb;
aditof::Status status;
static bool cameraStarted = false;
static std::string frameContent;
static aditof::FrameDetails frameDetalis;
aditof::Metadata *metaDataInfo = NULL;

// Number of bit shift neccessary to right to have BITS_PER_PIXEL defined pixel bit size
uint8_t depthShiftValue = 0;
uint8_t irShiftValue = 0;

// Message to Host
uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];

// This string is returned upon reqest from application. Needs data populated from available frame types of sensor
string availableModeDetails_prefix = "ft:";
string availableModeDetails = "";
// This string is returned upon reqest from application. Needs data populated with supported frame formats
string availableFormats_prefix = "format:";
string availableFormats = "";

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Send string message via libwebsocket
 * 
 * @param msg - Message to send
 * @param len - Length of message
 * @return int 
 */
int send_message(const char *msg, int len);

/**
 * @brief Send frame inbinary format via libwebsocket
 * 
 * @param msg Frame to send
 * @param len Length of message
 * @return int 
 */
int send_frame(const char *msg, int len);

/**
 * @brief Initilize libwebsocket for later communication
 * 
 * @param wsi Pointer to store access to socket conneciton
 */
void init_wsi_message(struct lws *wsi);

/**
 * @brief Send custom error message to HOST
 * 
 * @param msg Error message
 */
void send_error_message(std::string msg);

/**
 * @brief Initialize system, obtains camera list and initilizes camera with config file.
 * 
 * @param configFile - Config file for camera initialization
 * @return aditof::Status 
 */
aditof::Status initializeSystem(string configFile);

/**
 * @brief Reset camera and frame objects.
 * 
 * @return aditof::Status 
 */
aditof::Status deInitializeSystem();

/**
 * @brief Stop camera
 * 
 * @return aditof::Status 
 */
aditof::Status stopCamera();

/**
 * @brief Request one frame from camera and prapare for binary websocket transmission
 * 
 * @return aditof::Status 
 */
aditof::Status requestFrame();

/**
 * @brief Set frame type in camera object and sends response to user with frame widthxheight and available frame sizes
 * 
 * @param frameTypeNew - Frame type to set camera, must come from the available list
 * @return aditof::Status 
 */
aditof::Status setFrameType(std::string frameTypeNew);

/**
 * @brief Stores in global variable the format used for streaming
 * 
 * @param formatTypeNew - Value to store for format (possible values: "depth", "ab", "depth+ab")
 * @return aditof::Status 
 */
aditof::Status setFormat(std::string formatTypeNew);

/**
 * @brief Request an initial dummy frame from camera in order to obtain metadata information about the frame size and bits in depth and ab
 * 
 * @return aditof::Status 
 */
aditof::Status getMetadataInfo();

/**
 * @brief General callback function for socket-communication
 * 
 * @param wsi - Websockt pointer
 * @param reason - Reason for websocket message
 * @param user - User message is received from
 * @param in - Incomming message
 * @param len - Length of message
 * @return 0 in case of Success, different from 0 otherwise
 */
static int callback_tof(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len);

#endif
