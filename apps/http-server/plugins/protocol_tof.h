#ifndef HTTP_PROTOCOL_TOF
#define HTTP_PROTOCOL_TOF

#if !defined(LWS_PLUGIN_STATIC)
#define LWS_DLL
#define LWS_INTERNAL
#include <libwebsockets.h>
#endif

#define CHECK_PERIOD_US 50000
#define HEADER_SIZE 0
#define MAX_MESSAGE_LEN                                                        \
    (LWS_PRE + 1042 * 1024 * 2 + HEADER_SIZE) // Maximal pixel size

#include "mode_info.h"
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <algorithm>
#include <iostream>
#include <string>

using namespace std;
using namespace aditof;

struct pss__tof {
    int number;
};

struct vhd__tof {
    const unsigned int *options;
};

// Generic variables
struct lws *systemWsi;

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
ModeInfo::modeInfo modeInfo;

// To remove
bool qmp_size = false;

// Message to Host
uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];

// This string is returned upon reqest from application. Needs data populated from available frame types of sensor
string availableFrameTypes = "ft:sr-native,sr-qnative,lr-native,lr-qnative";
// This string is returned upon reqest from application. Needs data populated with supported frame formats
string availableFormats = "format:depth,ab,depth+ab";

string process_message(char *in) {
    std::string message = (const char *)in;
    int endOfString = message.find('\n');
    if (endOfString > 0)
        message = message.substr(0, endOfString);
    return message;
}

int send_message(const char *msg, int len);
int send_frame(const char *msg, int len);
void init_wsi_message(struct lws *wsi);
void send_error_message(std::string msg);
aditof::Status initializeSystem(string configFile);
aditof::Status deInitializeSystem();
aditof::Status stopCamera();
aditof::Status requestFrame();
aditof::Status setFrameType(std::string frameTypeNew);
aditof::Status setFormat(std::string formatTypeNew);
static int callback_tof(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len);

#endif