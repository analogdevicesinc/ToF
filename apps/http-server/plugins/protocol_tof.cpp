#include "protocol_tof.h"

int send_message(const char *msg, int len) {
    memcpy((void *)(buf + LWS_PRE), msg, len);
    int ret = lws_write(systemWsi, buf, LWS_PRE + len, LWS_WRITE_TEXT);
    if (!ret) {
        lwsl_err("ERROR writing to tof socket\n");
        return -1;
    }
    std::cout << "Response sent to Host\n";
    return 0;
}

int send_frame(const char *msg, int len) {
    memcpy((void *)(buf + LWS_PRE), msg, len);
    int ret = lws_write(systemWsi, buf, LWS_PRE + len, LWS_WRITE_BINARY);
    if (!ret) {
        lwsl_err("ERROR writing to tof socket\n");
        return -1;
    }
    std::cout << "Response sent to Host\n";
    return 0;
}

void init_wsi_message(struct lws *wsi) { systemWsi = wsi; }

void send_error_message(std::string msg) {
    msg = std::string("error_log:") + msg;
    send_message(msg.c_str(), msg.length());
}

aditof::Status initializeSystem(string configFile) {
    std::cout << "Initilize system \n";
    status = aditof::Status::OK;

    status = tofSystem.getCameraList(cameras);
    if (status != aditof::Status::OK) {
        cout << "Failed to get camera list!";
        return aditof::Status::GENERIC_ERROR;
    }

    camera = cameras.front();
    frame = std::make_shared<aditof::Frame>();

    status = camera->initialize(configFile);
    if (status != Status::OK) {
        cout << "Could not initialize camera!";
        return status;
    }

    return aditof::Status::OK;
}

aditof::Status deInitializeSystem() {
    std::cout << "Deinitialize system \n";
    status = aditof::Status::OK;

    if (camera) {
        camera.reset();
        frame.reset();
        return aditof::Status::OK;
    } else
        return aditof::Status::GENERIC_ERROR;
}

aditof::Status stopCamera() {
    std::cout << "Stop Camera\n";
    status = aditof::Status::OK;

    if (cameraStarted)
        status = camera->stop();
    if (status == aditof::Status::OK)
        cameraStarted = false;
    return status;
}

aditof::Status requestFrame() {
    status = aditof::Status::OK;

    if (!cameraStarted) {
        camera->start();
        cameraStarted = true;
    }
    status = camera->requestFrame(frame.get());
    if (status != aditof::Status::OK) {
        send_error_message(std::string("Error requesting frame"));
        return status;
    }

    if (frameContent == std::string("depth") ||
        frameContent == std::string("ab")) {

        status = frame->getData(frameContent, &frameData);

        if (status != aditof::Status::OK) {
            send_error_message(std::string("Failed to get frame data"));
            return status;
        }

        uint8_t *frameToSend =
            (uint8_t *)malloc(sizeof(uint8_t) * frameWidth * frameHeight);
        for (int i = 0; i < frameWidth * frameHeight; i++) {
            if (frameContent == std::string("depth"))
                if (qmp_size)
                    frameToSend[i] = (uint8_t)(frameData[i] / 256);
                else
                    frameToSend[i] = (uint8_t)(frameData[i] / 16);
            else
                frameToSend[i] = (uint8_t)((frameData[i]) & 0xff);
        }
        send_frame((const char *)frameToSend, frameWidth * frameHeight);
        free(frameToSend);
    } else if (frameContent == std::string("depth+ab")) {

        status = frame->getData(std::string("depth"), &frameDataDepth);
        status = frame->getData(std::string("ab"), &frameDataAb);

        if (status != aditof::Status::OK) {
            send_error_message(std::string("Failed to get frame data"));
            return status;
        }

        uint8_t *frameToSend =
            (uint8_t *)malloc(sizeof(uint8_t) * frameWidth * frameHeight * 2);

        for (int i = 0; i < frameWidth * frameHeight; i++) {
            // frameToSend[i] = (uint8_t)((frameDataDepth[i] >> 2) & 0xff);

            // preparing depth data
            if (qmp_size)
                frameToSend[i] = (uint8_t)(frameDataDepth[i] / 256);
            else
                frameToSend[i] = (uint8_t)(frameDataDepth[i] / 16);

            // preparing ir data
            frameToSend[i + frameWidth * frameHeight] =
                (uint8_t)((frameDataAb[i]) & 0xff);
        }

        send_frame((const char *)frameToSend, frameWidth * frameHeight * 2);
        free(frameToSend);

    } else {
        send_error_message(std::string("Unsupported data format type"));
        status = aditof::Status::GENERIC_ERROR;
    }
    return status;
}

aditof::Status setFrameType(std::string frameTypeNew) {
    status = aditof::Status::OK;

    status = camera->setFrameType(frameTypeNew);
    if (status != aditof::Status::OK) {
        send_error_message(std::string("Error while selecting frame type"));
        return status;
    }

    CameraDetails details;
    status = camera->getDetails(details);
    if (status != aditof::Status::OK) {
        send_error_message(
            std::string("Error while requesting camera details"));
        return status;
    }
    std::vector<FrameDataDetails> tmp_frameDetails =
        details.frameType.dataDetails;
    std::string dataType = "depth";

    auto detailsIter =
        std::find_if(tmp_frameDetails.begin(), tmp_frameDetails.end(),
                     [&dataType](const aditof::FrameDataDetails &details) {
                         return dataType == details.type;
                     });
    if (detailsIter == tmp_frameDetails.end()) {
        send_error_message(std::string("Error while requesting frame details"));
        return aditof::Status::GENERIC_ERROR;
    }

    frameWidth = detailsIter->width;
    frameHeight = detailsIter->height;

    std::cout << "Sending width and height: " << frameWidth << ", "
              << frameHeight << std::endl;

    std::string availableSize = "size:" + std::to_string(frameWidth) + "," +
                                std::to_string(frameHeight);

    // Send available size for the frame:
    send_message(availableSize.c_str(), availableSize.length());
    // Send available formats (depth/ab)
    send_message(availableFormats.c_str(), availableFormats.length());

    return status;
}

aditof::Status setFormat(std::string formatTypeNew) {
    frameContent = formatTypeNew;
    return aditof::Status::OK;
}

static int callback_tof(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
    struct pss__tof *pss = (struct pss__tof *)user;
    struct vhd__tof *vhd = (struct vhd__tof *)lws_protocol_vh_priv_get(
        lws_get_vhost(wsi), lws_get_protocol(wsi));
    uint8_t buf[LWS_PRE + MAX_MESSAGE_LEN], *p = &buf[LWS_PRE];
    int n, m;

    init_wsi_message(wsi);

    switch (reason) {
    case LWS_CALLBACK_PROTOCOL_INIT:
        vhd = (vhd__tof *)lws_protocol_vh_priv_zalloc(
            lws_get_vhost(wsi), lws_get_protocol(wsi), sizeof(struct vhd__tof));
        if (!vhd)
            return -1;
        break;

    case LWS_CALLBACK_ESTABLISHED:
        pss->number = 0;
        if (!vhd->options || !((*vhd->options) & 1))
            lws_set_timer_usecs(wsi, CHECK_PERIOD_US);
        break;

    case LWS_CALLBACK_SERVER_WRITEABLE:
        n = lws_snprintf((char *)p, sizeof(buf) - LWS_PRE, "%d", pss->number++);
        m = lws_write(wsi, p, n, LWS_WRITE_TEXT);
        if (m < n) {
            lwsl_err("ERROR %d writing to the socket\n", n);
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
                send_error_message(std::string("Unsupported camera type"));
                break;
            }

            // Send available frame types
            send_message(availableFrameTypes.c_str(),
                         availableFrameTypes.length());

        } else if (strncmp((const char *)in, "setft:", 6) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";
            int pos = message.find(":");
            std::string frameTypeNew = message.substr(pos + 1);

            // if (frameTypeNew == "sr-qnative" || frameTypeNew = "lr-qnative") //qnative -12, native 16 bits
            if (strcmp(frameTypeNew.c_str(), "sr-qnative") == 0 ||
                strcmp(frameTypeNew.c_str(), "lr-qnative") == 0)
                qmp_size = true;

            status = setFrameType(frameTypeNew);

        } else if (strncmp((const char *)in, "setFormat:", 9) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            int pos = message.find(":");
            std::string formatTypeNew = message.substr(pos + 1);

            status = setFormat(formatTypeNew);

            // Send state
            if (status == aditof::Status::OK) {
                string state = "ready\n";
                send_message(state.c_str(), state.length());
            }

        } else if (strncmp((const char *)in, "requestFrame\n", 13) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Messeage from HOST: " << message << "\n";

            status = requestFrame();

        } else if (strncmp((const char *)in, "stop\n", 5) == 0) {
            // Set chosen frame type
            std::string message = process_message((char *)in);
            std::cout << "Message from HOST: " << message << "\n";
            stopCamera();
        } else if (strncmp((const char *)in, "close\n", 6) == 0) {
            stopCamera();
            deInitializeSystem();
        } else if (strncmp((const char *)in, "reset\n", 6) == 0) {
            stopCamera();
            deInitializeSystem();
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
            lws_set_timer_usecs(wsi, CHECK_PERIOD_US);
        }
        break;

    default:
        break;
    }

    return 0;
}
