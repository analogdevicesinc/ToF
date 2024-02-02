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
#include "server.h"
#include "aditof/aditof.h"
#include "aditof/sensor_enumerator_factory.h"
#include "aditof/sensor_enumerator_interface.h"
#include "buffer.pb.h"

#include "../../sdk/src/connections/target/v4l_buffer_access_interface.h"

#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <iostream>
#include <linux/videodev2.h>
#include <map>
#include <queue>
#include <string>
#include <sys/time.h>

using namespace google::protobuf::io;

static const int FRAME_PREPADDING_BYTES = 2;
static int interrupted = 0;

/* Available sensors */
std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
bool sensors_are_created = false;
bool clientEngagedWithSensors = false;

std::unique_ptr<aditof::SensorEnumeratorInterface> sensorsEnumerator;

/* Server only works with one depth sensor */
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;
std::shared_ptr<aditof::V4lBufferAccessInterface> sensorV4lBufAccess;
int processedFrameSize;

static payload::ClientRequest buff_recv;
static payload::ServerResponse buff_send;

//sending frames separately without serializing it
uint8_t *buff_frame_to_send = NULL;
unsigned int buff_frame_length;
bool m_frame_ready = false;

static std::map<std::string, api_Values> s_map_api_Values;
static void Initialize();
void invoke_sdk_api(payload::ClientRequest buff_recv);
static bool Client_Connected = false;
static bool no_of_client_connected = false;
bool latest_sent_msg_is_was_buffered = false;
static std::queue<aditof::Adsd3500Status> adsd3500InterruptsQueue;

// A test mode that server can be set to. After sending one frame from sensor to host, it will repeat
// sending the same frame over and over without acquiring any other frame from sensor. This allows
// testing the network link speed because it eliminates operations on target such as getting the frame
// from v4l2 interface, passing the frame through depth compute and any deep copying.
static bool sameFrameEndlessRepeat = false;

struct clientData {
    bool hasFragments;
    std::vector<char> data;
};

static struct lws_protocols protocols[] = {
    {
        "network-protocol",
        Network::callback_function,
        sizeof(clientData),
        RX_BUFFER_BYTES,
    },
    {NULL, NULL, 0, 0} /* terminator */
};

aditof::SensorInterruptCallback callback = [](aditof::Adsd3500Status status) {
    adsd3500InterruptsQueue.push(status);
    DLOG(INFO) << "ADSD3500 interrupt occured: status = " << status;
};

static void cleanup_sensors() {
    camDepthSensor->adsd3500_unregister_interrupt_callback(callback);
    sensorV4lBufAccess.reset();
    camDepthSensor.reset();

    while (!adsd3500InterruptsQueue.empty()) {
        adsd3500InterruptsQueue.pop();
    }
    sensors_are_created = false;
    clientEngagedWithSensors = false;
}

Network ::Network() : context(nullptr) {}

int Network::callback_function(struct lws *wsi,
                               enum lws_callback_reasons reason, void *user,
                               void *in, size_t len) {
    uint n;

    switch (reason) {
    case LWS_CALLBACK_ESTABLISHED: {
        /*Check if another client is connected or not*/
        buff_send.Clear();
        if (Client_Connected == false) {
            std::cout << "Conn Established" << std::endl;
            Client_Connected = true;
            buff_send.set_message("Connection Allowed");
            lws_callback_on_writable(wsi);
            break;
        } else {
            std::cout << "Another client connected" << std::endl;
            no_of_client_connected = true;
            buff_send.set_message("Only 1 client connection allowed");
            lws_callback_on_writable(wsi);
        }
        break;
    }

    case LWS_CALLBACK_RECEIVE: {
#ifdef NW_DEBUG
        cout << endl << "Server has received data with len: " << len << endl;
#endif
        const size_t remaining = lws_remaining_packet_payload(wsi);
        bool isFinal = lws_is_final_fragment(wsi);

        struct clientData *clientData = static_cast<struct clientData *>(user);

        if (!remaining && isFinal) {
            if (clientData->hasFragments) {
                // apend message
                char *inData = static_cast<char *>(in);
                clientData->data.insert(clientData->data.end(), inData,
                                        inData + len);
                in = static_cast<void *>(clientData->data.data());
                len = clientData->data.size();
            }

            // process message
            google::protobuf::io::ArrayInputStream ais(in, len);
            CodedInputStream coded_input(&ais);

            buff_recv.ParseFromCodedStream(&coded_input);

            invoke_sdk_api(buff_recv);
            lws_callback_on_writable(wsi);

            clientData->data.clear();
            clientData->hasFragments = false;
        } else {
            // append message
            if (clientData->data.size() == 0) {
                clientData->data.reserve(len + remaining);
            }
            char *inData = static_cast<char *>(in);
            clientData->data.insert(clientData->data.end(), inData,
                                    inData + len);
            clientData->hasFragments = true;
        }

        break;
    }

    case LWS_CALLBACK_SERVER_WRITEABLE: {
        // TO INVESTIGATE: Currently this workaround prevents the server to send
        // the image buffer over and over again but as a side effect it lowers
        // the FPS with about 2-3 frames. How to avoid FPS reduction?
        if (latest_sent_msg_is_was_buffered) {
            latest_sent_msg_is_was_buffered = false;
            break;
        }
        unsigned int siz = 0;
        // Send the frame content without wrapping it in a protobuf message (speed optimization)
        if (m_frame_ready == true) {
            siz = buff_frame_length;

            n = lws_write(wsi, buff_frame_to_send + LWS_SEND_BUFFER_PRE_PADDING,
                          buff_frame_length, LWS_WRITE_TEXT);
            m_frame_ready = false;
            if (lws_partial_buffered(wsi)) {
                latest_sent_msg_is_was_buffered = true;
            }
        } else {
            siz = buff_send.ByteSize();
            unsigned char *pkt =
                new unsigned char[siz + LWS_SEND_BUFFER_PRE_PADDING];
            unsigned char *pkt_pad = pkt + LWS_SEND_BUFFER_PRE_PADDING;

            google::protobuf::io::ArrayOutputStream aos(pkt_pad, siz);
            CodedOutputStream *coded_output = new CodedOutputStream(&aos);
            buff_send.SerializeToCodedStream(coded_output);
            n = lws_write(wsi, pkt_pad, siz, LWS_WRITE_TEXT);

            if (lws_partial_buffered(wsi)) {
                latest_sent_msg_is_was_buffered = true;
            }

            delete coded_output;
            delete[] pkt;
        }
        if (n < 0)
            std::cout << "Error Sending" << std::endl;
        else if (n < siz)
            std::cout << "Partial write" << std::endl;
        else if (n == siz) {
        }
        break;
    }

    case LWS_CALLBACK_CLOSED: {
        if (Client_Connected == true && no_of_client_connected == false) {
            /*CONN_CLOSED event is for first and only client connected*/
            std::cout << "Connection Closed" << std::endl;

            Client_Connected = false;
            break;
        } else {
            /*CONN_CLOSED event for more than 1 client connected */
            std::cout << "Another Client Connection Closed" << std::endl;
            no_of_client_connected = false;
            break;
        }
    }

    default: {
#ifdef NW_DEBUG
        cout << reason << endl;
#endif
    } break;
    }

    return 0;
}

void sigint_handler(int) { interrupted = 1; }

int main(int argc, char *argv[]) {

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    LOG(INFO) << "Server built with websockets version:" << LWS_LIBRARY_VERSION;

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));

    info.port = 5000;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.pt_serv_buf_size = 4096;
    std::unique_ptr<Network> network(new Network);

    network->context = lws_create_context(&info);

    Initialize();
    int msTimeout;
// TO DO: After 6-12 months we should remove this #if-else and keep only things related to 3.2.3
#if LWS_LIBRARY_VERSION_NUMBER > 3002003
    msTimeout = 0;
#else
    msTimeout = 50;
#endif

#if 0
  /* Note: Simply enabling this won't work, need libwebsocket compiled differently to demonize this */
  if(lws_daemonize("/tmp/server_lock"))
  {
    fprintf(stderr,"Failed to daemonize\n");
  }
#endif

    while (!interrupted) {
        lws_service(network->context, msTimeout /* timeout_ms */);
    }

    if (sensors_are_created) {
        cleanup_sensors();
    }

    lws_context_destroy(network->context);

    return 0;
}

void invoke_sdk_api(payload::ClientRequest buff_recv) {
    buff_send.Clear();
    buff_send.set_server_status(::payload::ServerStatus::REQUEST_ACCEPTED);

    DLOG(INFO) << buff_recv.func_name() << " function";

    switch (s_map_api_Values[buff_recv.func_name()]) {

    case FIND_SENSORS: {
        // Check if client didn't hang up (why would want to search for sensor if it already tried to open one)
        if (clientEngagedWithSensors) {
            cleanup_sensors();
        }

        if (!sensors_are_created) {
            sensorsEnumerator =
                aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
            if (!sensorsEnumerator) {
                std::string errMsg =
                    "Failed to create a target sensor enumerator";
                LOG(WARNING) << errMsg;
                buff_send.set_message(errMsg);
                buff_send.set_status(static_cast<::payload::Status>(
                    aditof::Status::UNAVAILABLE));
                break;
            }

            sensorsEnumerator->searchSensors();
            sensorsEnumerator->getDepthSensors(depthSensors);
            sensors_are_created = true;
        }

        /* Add information about available sensors */

        // Depth sensor
        if (depthSensors.size() < 1) {
            buff_send.set_message("No depth sensors are available");
            buff_send.set_status(::payload::Status::UNREACHABLE);
            break;
        }

        camDepthSensor = depthSensors.front();
        auto pbSensorsInfo = buff_send.mutable_sensors_info();
        sensorV4lBufAccess =
            std::dynamic_pointer_cast<aditof::V4lBufferAccessInterface>(
                camDepthSensor);

        std::string name;
        camDepthSensor->getName(name);
        auto pbDepthSensorInfo = pbSensorsInfo->mutable_image_sensors();
        pbDepthSensorInfo->set_name(name);

        std::string kernelversion;
        std::string ubootversion;
        std::string sdversion;
        auto cardVersion = buff_send.mutable_card_image_version();

        sensorsEnumerator->getKernelVersion(kernelversion);
        cardVersion->set_kernelversion(kernelversion);
        sensorsEnumerator->getUbootVersion(ubootversion);
        cardVersion->set_ubootversion(ubootversion);
        sensorsEnumerator->getSdVersion(sdversion);
        cardVersion->set_sdversion(sdversion);

        // This server is now subscribing for interrupts of ADSD3500
        aditof::Status registerCbStatus =
            camDepthSensor->adsd3500_register_interrupt_callback(callback);
        if (registerCbStatus != aditof::Status::OK) {
            LOG(WARNING) << "Could not register callback";
            // TBD: not sure whether to send this error to client or not
        }

        buff_send.set_status(
            static_cast<::payload::Status>(aditof::Status::OK));
        break;
    }

    case OPEN: {
        aditof::Status status = camDepthSensor->open();
        buff_send.set_status(static_cast<::payload::Status>(status));
        clientEngagedWithSensors = true;
        break;
    }

    case START: {
        aditof::Status status = camDepthSensor->start();

        // When in test mode, capture 2 frames. 1st might be corrupt after a ADSD3500 reset.
        // 2nd frame will be the one sent over and over again by server in test mode.
        if (sameFrameEndlessRepeat) {
            for (int i = 0; i < 2; ++i) {
                status = camDepthSensor->getFrame(
                    (uint16_t *)(buff_frame_to_send +
                                 LWS_SEND_BUFFER_PRE_PADDING +
                                 FRAME_PREPADDING_BYTES));
                if (status != aditof::Status::OK) {
                    LOG(ERROR) << "Failed to get frame!";
                }
            }
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case STOP: {
        aditof::Status status = camDepthSensor->stop();
        buff_send.set_status(static_cast<::payload::Status>(status));

        break;
    }

    case GET_AVAILABLE_FRAME_TYPES: {
        std::vector<aditof::DepthSensorFrameType> aditofFrameTypes;

        aditof::Status status =
            camDepthSensor->getAvailableFrameTypes(aditofFrameTypes);
        for (const auto &aditofType : aditofFrameTypes) {
            auto protoType = buff_send.add_available_frame_types();

            protoType->set_width(aditofType.width);
            protoType->set_height(aditofType.height);
            protoType->set_type(aditofType.type);
            for (const auto &aditofContent : aditofType.content) {
                auto protoContent = protoType->add_depthsensorframecontent();
                protoContent->set_type(aditofContent.type);
                protoContent->set_width(aditofContent.width);
                protoContent->set_height(aditofContent.height);
            }
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_FRAME_TYPE: {
        aditof::DepthSensorFrameType aditofFrameType;

        aditofFrameType.width = buff_recv.frame_type().width();
        aditofFrameType.height = buff_recv.frame_type().height();
        aditofFrameType.type = buff_recv.frame_type().type();
        for (int i = 0;
             i < buff_recv.frame_type().depthsensorframecontent().size(); ++i) {
            aditof::DepthSensorFrameContent aditofFrameContent;

            aditofFrameContent.type =
                buff_recv.frame_type().depthsensorframecontent(i).type();
            aditofFrameContent.width =
                buff_recv.frame_type().depthsensorframecontent(i).width();
            aditofFrameContent.height =
                buff_recv.frame_type().depthsensorframecontent(i).height();
            aditofFrameType.content.emplace_back(aditofFrameContent);
        }

        aditof::Status status = camDepthSensor->setFrameType(aditofFrameType);

        if (status == aditof::Status::OK) {
            //width * height * 2 bytes/pixel * 2 frames(depth/ab)
            //Looking for depth resolution
            int width_tmp = aditofFrameType.width;
            int height_tmp = aditofFrameType.height;

            for (auto it = aditofFrameType.content.begin();
                 it != aditofFrameType.content.end(); ++it) {
                if (!(it->type.compare(std::string("depth")))) {
                    width_tmp = it->width;
                    height_tmp = it->height;
                }
            }

            if (aditofFrameType.type == "pcm-native") {
                processedFrameSize = width_tmp * height_tmp;
            } else {
                processedFrameSize = width_tmp * height_tmp * 4;
            }

            if (buff_frame_to_send != nullptr) {
                delete[] buff_frame_to_send;
                buff_frame_to_send = nullptr;
            }
            buff_frame_to_send =
                new uint8_t[LWS_SEND_BUFFER_PRE_PADDING +
                            FRAME_PREPADDING_BYTES +
                            processedFrameSize * sizeof(uint16_t)];
            buff_frame_length = processedFrameSize * 2 + FRAME_PREPADDING_BYTES;
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case PROGRAM: {
        size_t programSize = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint8_t *pdata = reinterpret_cast<const uint8_t *>(
            buff_recv.func_bytes_param(0).c_str());
        aditof::Status status = camDepthSensor->program(pdata, programSize);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_FRAME: {
        aditof::Status status = aditof::Status::OK;

        if (sameFrameEndlessRepeat) {
            m_frame_ready = true;
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        //TO DO: get value of m_depthComputeOnTarget from sensor
        status = camDepthSensor->getFrame(
            (uint16_t *)(buff_frame_to_send + LWS_SEND_BUFFER_PRE_PADDING +
                         FRAME_PREPADDING_BYTES));
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Failed to get frame!";
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        uint8_t *pInterruptOccuredByte =
            &buff_frame_to_send[LWS_SEND_BUFFER_PRE_PADDING +
                                0]; // 1st byte after LWS prepadding bytes
        if (!adsd3500InterruptsQueue.empty()) {
            *pInterruptOccuredByte = 123;
        } else {
            *pInterruptOccuredByte = 0;
        }

        m_frame_ready = true;

        buff_send.set_status(payload::Status::OK);
        break;

        status = sensorV4lBufAccess->waitForBuffer();

        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        struct v4l2_buffer buf;

        status = sensorV4lBufAccess->dequeueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        uint8_t *buffer;

        status = sensorV4lBufAccess->getInternalBuffer(&buffer,
                                                       buff_frame_length, buf);
        if (buff_frame_to_send != NULL) {
            free(buff_frame_to_send);
            buff_frame_to_send = NULL;
        }
        buff_frame_to_send =
            (uint8_t *)malloc((buff_frame_length + LWS_SEND_BUFFER_PRE_PADDING +
                               FRAME_PREPADDING_BYTES) *
                              sizeof(uint8_t));

        memcpy(buff_frame_to_send + LWS_SEND_BUFFER_PRE_PADDING +
                   FRAME_PREPADDING_BYTES,
               buffer, buff_frame_length * sizeof(uint8_t));

        m_frame_ready = true;

        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        status = sensorV4lBufAccess->enqueueInternalBuffer(buf);
        if (status != aditof::Status::OK) {
            buff_send.set_status(static_cast<::payload::Status>(status));
            break;
        }

        buff_send.set_status(payload::Status::OK);
        break;
    }

    case READ_REGISTERS: {
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        uint16_t *data = new uint16_t[length];
        bool burst = static_cast<bool>(buff_recv.func_int32_param(1));
        aditof::Status status =
            camDepthSensor->readRegisters(address, data, length, burst);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, length * sizeof(uint16_t));
        }
        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case WRITE_REGISTERS: {
        size_t length = static_cast<size_t>(buff_recv.func_int32_param(0));
        const uint16_t *address = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(0).c_str());
        const uint16_t *data = reinterpret_cast<const uint16_t *>(
            buff_recv.func_bytes_param(1).c_str());
        bool burst = static_cast<bool>(buff_recv.func_int32_param(1));
        aditof::Status status =
            camDepthSensor->writeRegisters(address, data, length, burst);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_AVAILABLE_CONTROLS: {
        std::vector<std::string> aditofControls;

        aditof::Status status =
            camDepthSensor->getAvailableControls(aditofControls);
        for (const auto &aditofControl : aditofControls) {
            buff_send.add_strings_payload(aditofControl);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_CONTROL: {
        std::string controlName = buff_recv.func_strings_param(0);
        std::string controlValue = buff_recv.func_strings_param(1);
        aditof::Status status =
            camDepthSensor->setControl(controlName, controlValue);
        if (controlName == "netlinktest") {
            sameFrameEndlessRepeat = controlValue == "1";
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_CONTROL: {
        std::string controlName = buff_recv.func_strings_param(0);
        std::string controlValue;
        aditof::Status status =
            camDepthSensor->getControl(controlName, controlValue);
        buff_send.add_strings_payload(controlValue);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case INIT_TARGET_DEPTH_COMPUTE: {
        aditof::Status status = camDepthSensor->initTargetDepthCompute(
            (uint8_t *)buff_recv.func_bytes_param(0).c_str(),
            static_cast<uint16_t>(buff_recv.func_int32_param(0)),
            (uint8_t *)buff_recv.func_bytes_param(1).c_str(),
            static_cast<uint16_t>(buff_recv.func_int32_param(1)));

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_CMD: {
        uint16_t cmd = static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint16_t data;
        unsigned int usDelay =
            static_cast<unsigned int>(buff_recv.func_int32_param(1));

        aditof::Status status =
            camDepthSensor->adsd3500_read_cmd(cmd, &data, usDelay);
        if (status == aditof::Status::OK) {
            buff_send.add_int32_payload(static_cast<::google::int32>(data));
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_CMD: {
        uint16_t cmd = static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint16_t data = static_cast<uint16_t>(buff_recv.func_int32_param(1));

        aditof::Status status = camDepthSensor->adsd3500_write_cmd(cmd, data);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(1));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(),
               4 * sizeof(uint8_t));
        aditof::Status status =
            camDepthSensor->adsd3500_read_payload_cmd(cmd, data, payload_len);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, payload_len);
        }

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_READ_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint8_t *data = new uint8_t[payload_len];

        aditof::Status status =
            camDepthSensor->adsd3500_read_payload(data, payload_len);
        if (status == aditof::Status::OK) {
            buff_send.add_bytes_payload(data, payload_len);
        }

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_PAYLOAD_CMD: {
        uint32_t cmd = static_cast<uint32_t>(buff_recv.func_int32_param(0));
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(1));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(), payload_len);
        aditof::Status status =
            camDepthSensor->adsd3500_write_payload_cmd(cmd, data, payload_len);

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_WRITE_PAYLOAD: {
        uint16_t payload_len =
            static_cast<uint16_t>(buff_recv.func_int32_param(0));
        uint8_t *data = new uint8_t[payload_len];

        memcpy(data, buff_recv.func_bytes_param(0).c_str(), payload_len);
        aditof::Status status =
            camDepthSensor->adsd3500_write_payload(data, payload_len);

        delete[] data;
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case ADSD3500_GET_STATUS: {
        int chipStatus;
        int imagerStatus;

        aditof::Status status =
            camDepthSensor->adsd3500_get_status(chipStatus, imagerStatus);
        if (status == aditof::Status::OK) {
            buff_send.add_int32_payload(chipStatus);
            buff_send.add_int32_payload(imagerStatus);
        }

        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case GET_INTERRUPTS: {
        while (!adsd3500InterruptsQueue.empty()) {
            buff_send.add_int32_payload((int)adsd3500InterruptsQueue.front());
            adsd3500InterruptsQueue.pop();
        }

        buff_send.set_status(
            static_cast<::payload::Status>(aditof::Status::OK));
        break;
    }

    case HANG_UP: {
        if (sensors_are_created) {
            cleanup_sensors();
        }

        break;
    }

    case GET_INI_PARAM: {
        std::map<std::string, float> ini_params;
        aditof::Status status = camDepthSensor->getIniParams(ini_params);
        if (status == aditof::Status::OK) {
            buff_send.add_float_payload(ini_params["ab_thresh_min"]);
            buff_send.add_float_payload(ini_params["ab_sum_thresh"]);
            buff_send.add_float_payload(ini_params["conf_thresh"]);
            buff_send.add_float_payload(ini_params["radial_thresh_min"]);
            buff_send.add_float_payload(ini_params["radial_thresh_max"]);
            buff_send.add_float_payload(ini_params["jblf_apply_flag"]);
            buff_send.add_float_payload(ini_params["jblf_window_size"]);
            buff_send.add_float_payload(ini_params["jblf_gaussian_sigma"]);
            buff_send.add_float_payload(ini_params["jblf_exponential_term"]);
            buff_send.add_float_payload(ini_params["jblf_max_edge"]);
            buff_send.add_float_payload(ini_params["jblf_ab_threshold"]);
            buff_send.add_float_payload(ini_params["headerSize"]);
        }
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    case SET_INI_PARAM: {
        std::map<std::string, float> ini_params;
        ini_params["ab_thresh_min"] = buff_recv.func_float_param(0);
        ini_params["ab_sum_thresh"] = buff_recv.func_float_param(1);
        ini_params["conf_thresh"] = buff_recv.func_float_param(2);
        ini_params["radial_thresh_min"] = buff_recv.func_float_param(3);
        ini_params["radial_thresh_max"] = buff_recv.func_float_param(4);
        ini_params["jblf_apply_flag"] = buff_recv.func_float_param(5);
        ini_params["jblf_window_size"] = buff_recv.func_float_param(6);
        ini_params["jblf_gaussian_sigma"] = buff_recv.func_float_param(7);
        ini_params["jblf_exponential_term"] = buff_recv.func_float_param(8);
        ini_params["jblf_max_edge"] = buff_recv.func_float_param(9);
        ini_params["jblf_ab_threshold"] = buff_recv.func_float_param(10);

        aditof::Status status = camDepthSensor->setIniParams(ini_params);
        buff_send.set_status(static_cast<::payload::Status>(status));
        break;
    }

    default: {
        std::string msgErr = "Function not found";
        std::cout << msgErr << "\n";

        buff_send.set_message(msgErr);
        buff_send.set_server_status(::payload::ServerStatus::REQUEST_UNKNOWN);
        break;
    }
    } // switch

    buff_send.set_interrupt_occured(!adsd3500InterruptsQueue.empty());

    buff_recv.Clear();
}

void Initialize() {
    s_map_api_Values["FindSensors"] = FIND_SENSORS;
    s_map_api_Values["Open"] = OPEN;
    s_map_api_Values["Start"] = START;
    s_map_api_Values["Stop"] = STOP;
    s_map_api_Values["GetAvailableFrameTypes"] = GET_AVAILABLE_FRAME_TYPES;
    s_map_api_Values["SetFrameType"] = SET_FRAME_TYPE;
    s_map_api_Values["Program"] = PROGRAM;
    s_map_api_Values["GetFrame"] = GET_FRAME;
    s_map_api_Values["ReadRegisters"] = READ_REGISTERS;
    s_map_api_Values["WriteRegisters"] = WRITE_REGISTERS;
    s_map_api_Values["GetAvailableControls"] = GET_AVAILABLE_CONTROLS;
    s_map_api_Values["SetControl"] = SET_CONTROL;
    s_map_api_Values["GetControl"] = GET_CONTROL;
    s_map_api_Values["InitTargetDepthCompute"] = INIT_TARGET_DEPTH_COMPUTE;
    s_map_api_Values["Adsd3500ReadCmd"] = ADSD3500_READ_CMD;
    s_map_api_Values["Adsd3500WriteCmd"] = ADSD3500_WRITE_CMD;
    s_map_api_Values["Adsd3500ReadPayloadCmd"] = ADSD3500_READ_PAYLOAD_CMD;
    s_map_api_Values["Adsd3500ReadPayload"] = ADSD3500_READ_PAYLOAD;
    s_map_api_Values["Adsd3500WritePayloadCmd"] = ADSD3500_WRITE_PAYLOAD_CMD;
    s_map_api_Values["Adsd3500WritePayload"] = ADSD3500_WRITE_PAYLOAD;
    s_map_api_Values["Adsd3500GetStatus"] = ADSD3500_GET_STATUS;
    s_map_api_Values["GetInterrupts"] = GET_INTERRUPTS;
    s_map_api_Values["HangUp"] = HANG_UP;
    s_map_api_Values["GetIniParam"] = GET_INI_PARAM;
    s_map_api_Values["SetIniParam"] = SET_INI_PARAM;
}
