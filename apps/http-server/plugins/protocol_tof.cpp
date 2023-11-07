#if !defined(LWS_PLUGIN_STATIC)
#define LWS_DLL
#define LWS_INTERNAL
#include <libwebsockets.h>
#endif
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <iostream>
#include <string>
using namespace std;
using namespace aditof;

#define DUMB_PERIOD_US 50000
#define HEADER_SIZE 0
#define MAX_MESSAGE_LEN (1042 * 1024 * 2 + HEADER_SIZE) // For Megapixel

struct pss__tof {
    int number;
};

struct vhd__tof {
    const unsigned int *options;
};

//To remove
#include <fstream>

//Tof variables
aditof::System tofSystem;
std::vector<std::shared_ptr<aditof::Camera>> cameras;
shared_ptr<aditof::Camera> camera;

// Frame related variables
static aditof::Frame frame;
uint16_t frameWidth;
uint16_t frameHeight;
uint16_t *frameData = NULL;
uint16_t *frameDataDepth;
uint16_t *frameDataIr;
// uint8_t *frameToSend = NULL;

aditof::Status status;
static bool cameraStarted = false;
static std::string frameContent;
static aditof::FrameDetails frameDetalis;
static int minRange = 30;
static int maxRange = 4000;

//ToF init function
int initializeSystem(string configFile) {
    status = tofSystem.getCameraList(cameras);
    if (status != aditof::Status::OK) {
        cout << "Failed to get camera list!";
        return -1;
    }

    camera = cameras.front();

    status = camera->setControl("initialization_config", configFile);
    if (status != Status::OK) {
        cout << "Failed to set control!";
        return -1;
    }

    status = camera->initialize();
    if (status != Status::OK) {
        cout << "Could not initialize camera!";
        return -1;
    }

    return 0;
}

//Message to Host
uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];

//Available frametypes
string availableFrameTypes = "ft:sr-native,sr-qnative,lr-native,lr-qnative";
string availableFormats = "format:depth,ir,depth+ir";

/**
 * according to protocol take into consideration messages until the first '\n' character
 * otherwise eras everything else
*/

string process_message(char *in) {
    std::string message = (const char *)in;
    int endOfString = message.find('\n');
    if (endOfString > 0)
        message = message.substr(0, endOfString);
    return message;
}

int send_message(struct lws *wsi, const char *msg, int len) {
    memcpy((void *)(buf + LWS_PRE), msg, len);
    int ret = lws_write(wsi, buf, LWS_PRE + len, LWS_WRITE_TEXT);
    if (!ret) {
        lwsl_err("ERROR writing to tof socket\n");
        return -1;
    }
    std::cout << "Response sent to Host\n";
    return 0;
}

int send_frame(struct lws *wsi, const char *msg, int len) {
    memcpy((void *)(buf + LWS_PRE), msg, len);
    int ret = lws_write(wsi, buf, LWS_PRE + len, LWS_WRITE_BINARY);
    if (!ret) {
        lwsl_err("ERROR writing to tof socket\n");
        return -1;
    }
    std::cout << "Response sent to Host\n";
    return 0;
}

void send_error_message(struct lws *wsi, std::string msg) {
    msg = std::string("error_log:") + msg;
    send_message(wsi, msg.c_str(), msg.length());
}

static int callback_tof(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
    struct pss__tof *pss = (struct pss__tof *)user;
    struct vhd__tof *vhd = (struct vhd__tof *)lws_protocol_vh_priv_get(
        lws_get_vhost(wsi), lws_get_protocol(wsi));
    uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];
    const struct lws_protocol_vhost_options *opt;
    int n, m;

    switch (reason) {
    case LWS_CALLBACK_PROTOCOL_INIT:
        vhd = (vhd__tof *)lws_protocol_vh_priv_zalloc(
            lws_get_vhost(wsi), lws_get_protocol(wsi), sizeof(struct vhd__tof));
        if (!vhd)
            return -1;
        if ((opt = lws_pvo_search((const struct lws_protocol_vhost_options *)in,
                                  "options")))
            vhd->options = (unsigned int *)opt->value;
        break;

    case LWS_CALLBACK_ESTABLISHED:
        pss->number = 0;
        if (!vhd->options || !((*vhd->options) & 1))
            lws_set_timer_usecs(wsi, DUMB_PERIOD_US);
        break;

    case LWS_CALLBACK_SERVER_WRITEABLE:
        n = lws_snprintf((char *)p, sizeof(buf) - LWS_PRE, "%d", pss->number++);
        m = lws_write(wsi, p, n, LWS_WRITE_TEXT);
        if (m < n) {
            lwsl_err("ERROR %d writing to di socket\n", n);
            return -1;
        }
        break;

    case LWS_CALLBACK_RECEIVE:
        if (len < 5)
            break;

        if (strncmp((const char *)in, "reset\n", 6) == 0) {
            pss->number = 0;
            std::cout << "Resetting counter \n";

        } else if (strncmp((const char *)in, "open:", 5) == 0) {
            // Initialize camera
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";
            if (message == "open:adsd3100") {
                initializeSystem("config/config_adsd3500_adsd3100.json");
            } else if (message == "open:adsd3030") {
                initializeSystem("config/config_adsd3500_adsd3030.json");
            } else {
                send_error_message(wsi, std::string("Unsupported camera type"));
                break;
            }

            // Send available frame types
            send_message(wsi, availableFrameTypes.c_str(),
                         availableFrameTypes.length());

        } else if (strncmp((const char *)in, "setft:", 6) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            int pos = message.find(":");
            status = camera->setFrameType(message.substr(pos + 1));
            if (status != aditof::Status::OK) {
                send_error_message(
                    wsi, std::string("Error while selecting frame type"));
                break;
            }
            status = camera->requestFrame(&frame);
            if (status != aditof::Status::OK) {
                send_error_message(wsi,
                                   std::string("Error while requesting frame"));
                //     SOLVE THIS !!!!
                // break;
            }
            status = frame.getDetails(frameDetalis);
            if (status != aditof::Status::OK) {
                send_error_message(
                    wsi, std::string("Error while getting frame details"));
                break;
            }
            frameWidth = frameDetalis.width;
            frameHeight = frameDetalis.height;

            std::string availableSize = "size:" + std::to_string(frameWidth) +
                                        "," + std::to_string(frameHeight);

            // Send available size for the frame:
            send_message(wsi, availableSize.c_str(), availableSize.length());
            // Send available formats (depth/ir)
            send_message(wsi, availableFormats.c_str(),
                         availableFormats.length());

        } else if (strncmp((const char *)in, "setFormat:", 9) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            int pos = message.find(":");
            frameContent = message.substr(pos + 1);

            // Send state
            string state = "ready\n";
            send_message(wsi, state.c_str(), state.length());

        } else if (strncmp((const char *)in, "requestFrame\n", 13) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";
            if (!cameraStarted) {
                camera->start();
                cameraStarted = true;
            }
            status = camera->requestFrame(&frame);
            if (status != aditof::Status::OK) {
                send_error_message(wsi, std::string("Error requesting frame"));
                break;
            }

            if (frameContent == std::string("depth") ||
                frameContent == std::string("ir")) {

                status = frame.getData(frameContent, &frameData);

                if (status != aditof::Status::OK) {
                    send_error_message(wsi,
                                       std::string("Failed to get frame data"));
                }

                uint8_t *frameToSend = (uint8_t *)malloc(
                    sizeof(uint8_t) * frameWidth * frameHeight);
                for (int i = 0; i < frameWidth * frameHeight; i++) {
                    if (frameContent == std::string("depth"))
                        frameToSend[i] = (uint8_t)((frameData[i] >> 2) & 0xff);
                    else
                        frameToSend[i] = (uint8_t)((frameData[i] >> 4) & 0xff);
                }
                send_frame(wsi, (const char *)frameToSend, MAX_MESSAGE_LEN);
                free(frameToSend);
            } else if (frameContent == std::string("depth+ir")) {

                status = frame.getData(std::string("depth"), &frameDataDepth);
                status = frame.getData(std::string("ir"), &frameDataIr);

                if (status != aditof::Status::OK) {
                    send_error_message(wsi,
                                       std::string("Failed to get frame data"));
                }

                uint8_t *frameToSend = (uint8_t *)malloc(
                    sizeof(uint8_t) * frameWidth * frameHeight * 2);

                for (int i = 0; i < frameWidth * frameHeight; i++) {
                    frameToSend[i] = (uint8_t)((frameDataDepth[i] >> 2) & 0xff);
                    frameToSend[i + frameWidth * frameHeight] =
                        (uint8_t)((frameDataIr[i] >> 4) & 0xff);
                }

                send_frame(wsi, (const char *)frameToSend, MAX_MESSAGE_LEN);
                free(frameToSend);

            } else {
                send_error_message(wsi,
                                   std::string("Unsupported data format type"));
            }

        } else if (strncmp((const char *)in, "stop\n", 5) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Message from HOST: " << message << "\n";

            status = camera->stop();
            cameraStarted = false;

        } else if (strncmp((const char *)in, "close\n", 6) == 0) {
            //close camera
        } else if (strncmp((const char *)in, "closeme\n", 8) == 0) {
            lwsl_notice("dumb_inc: closing as requested\n");
            lws_close_reason(wsi, LWS_CLOSE_STATUS_GOINGAWAY,
                             (unsigned char *)"seeya", 5);
            return -1;
        }
        break;

    case LWS_CALLBACK_TIMER:
        if (!vhd->options || !((*vhd->options) & 1)) {
            lws_callback_on_writable_all_protocol_vhost(lws_get_vhost(wsi),
                                                        lws_get_protocol(wsi));
            lws_set_timer_usecs(wsi, DUMB_PERIOD_US);
        }
        break;

    default:
        break;
    }

    return 0;
}
