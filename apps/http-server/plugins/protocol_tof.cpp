#if !defined(LWS_PLUGIN_STATIC)
#define LWS_DLL
#define LWS_INTERNAL
#include <libwebsockets.h>
#endif
#include <iostream>
#include <string>
using namespace std;

#define DUMB_PERIOD_US 50000
#define MAX_MESSAGE_LEN (524288 + 100) // For Quarter Megapixel

struct pss__tof {
    int number;
};

struct vhd__tof {
    const unsigned int *options;
};

//To remove
#include <fstream>

//Tof variables
//Message to Host
uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];
//Camera initialized
void *camera;
//Available frametypes
string availableFrameTypes = "ft:sr-mp,sr-qmp,lr-mp,lr-qmp,pcm,n-mp,n-qmp";
string availableFormats = "format:depth,ir";

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
    memcpy((void *)(buf), msg, len);
    int ret = lws_write(wsi, buf, len, LWS_WRITE_BINARY);
    if (!ret) {
        lwsl_err("ERROR writing to tof socket\n");
        return -1;
    }
    std::cout << "Response sent to Host\n";
    return 0;
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

            // Send available frame types
            send_message(wsi, availableFrameTypes.c_str(),
                         availableFrameTypes.length());

        } else if (strncmp((const char *)in, "setft:", 6) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            // Send available formats (depth/ir)
            send_message(wsi, availableFormats.c_str(),
                         availableFormats.length());

        } else if (strncmp((const char *)in, "setFormat:", 9) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            // Send state
            string state = "ready\n";
            send_message(wsi, state.c_str(), state.length());

        } else if (strncmp((const char *)in, "start\n", 6) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            // To remove
            std::cout << "Reding in binary file\n";
            char *buffer;
            long size;
            ifstream file(
                "/home/analog/Workspace/ToF/apps/http-server/out_ir_.bin",
                ios::in | ios::binary | ios::ate);
            size = file.tellg();
            file.seekg(0, ios::beg);
            buffer = new char[size];
            file.read(buffer, size);
            file.close();
            std::cout << "Read in: " << size << "\n";
            // const char buff2 = (1, 2, 3);
            // send_frame(wsi, &buff2, 3);
            send_frame(wsi, buffer, size);

        } else if (strncmp((const char *)in, "stop\n", 5) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

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
